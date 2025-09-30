#include "pure_pursuit.h"
#include <cmath>
#include <algorithm>

PurePursuit::PurePursuit(Chassis& chassis, double lookahead_radius, double min_speed, double max_speed, double curvature_gain)
    : chassis(chassis), lookahead_radius(lookahead_radius), last_lookahead_idx(0),
      min_speed(min_speed), max_speed(max_speed), curvature_gain(curvature_gain) {}

void PurePursuit::set_path(const std::vector<Pose>& waypoints) {
    path = waypoints;
    last_lookahead_idx = 0;
}

double PurePursuit::distance(const Pose& a, const Pose& b) const {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::optional<Pose> PurePursuit::get_lookahead(const Pose& robot_pose) {
    for (std::size_t i = last_lookahead_idx; i < path.size(); ++i) {
        if (distance(robot_pose, path[i]) >= lookahead_radius) {
            last_lookahead_idx = i;
            return path[i];
        }
    }
    if (!path.empty())
        return path.back();
    return std::nullopt;
}

// Simplistic curvature calculation for the arc to the lookahead point
double PurePursuit::calculate_curvature(const Pose& robot_pose, const Pose& lookahead) const {
    double dx = lookahead.x - robot_pose.x;
    double dy = lookahead.y - robot_pose.y;
    double lookahead_dist = std::sqrt(dx * dx + dy * dy);

    // Transform lookahead point to robot-relative coordinates
    double angle_to_lookahead = std::atan2(dy, dx);
    double heading_error = angle_to_lookahead - robot_pose.heading;
    // Normalize heading error
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;

    // Pure Pursuit curvature formula
    // kappa = 2 * sin(heading_error) / lookahead_distance
    if (lookahead_dist < 1e-6) return 0.0;
    return 2.0 * std::sin(heading_error) / lookahead_dist;
}

double position_tolerance = 2.0; // units (e.g., inches or cm)
double heading_tolerance = 0.4; 

bool PurePursuit::step() {
    Pose robot_pose = chassis.get_pose();
    auto lookahead = get_lookahead(robot_pose);

    // Check if we are close enough to the final waypoint
    double final_dist = distance(robot_pose, path.back());
    double final_heading_err = std::abs(robot_pose.heading - path.back().heading);
    if (final_dist < position_tolerance && final_heading_err < heading_tolerance) {
        chassis.tank(0, 0); // Stop motors
        return true;
    }

    if (!lookahead)
        return true;

    double dx = lookahead->x - robot_pose.x;
    double dy = lookahead->y - robot_pose.y;
    double target_angle = std::atan2(dy, dx);

    double heading_error = target_angle - robot_pose.heading;
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;

    // Curvature-based speed control
    double curvature = calculate_curvature(robot_pose, *lookahead);
    double speed = max_speed / (1.0 + curvature_gain * std::abs(curvature));
    speed = std::clamp(speed, min_speed, max_speed);

    double angular_speed = heading_error * 50; // Tune gain as needed

    chassis.tank(speed + angular_speed, speed - angular_speed);

    return distance(robot_pose, path.back()) < lookahead_radius;
}

void PurePursuit::start(uint32_t delay_ms) {
    while (!step()) {
        pros::delay(delay_ms);
    }
    chassis.tank(0, 0);
}