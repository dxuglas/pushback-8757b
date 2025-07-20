#include "odometry.h"
#include "pros/imu.hpp"
#include "tracking_wheel.h"
#include "utils/angle.h"
#include "utils/pose.h"
#include <cmath>
#include <optional>
#include <vector>

struct TrackingWheelData {
    double distance;
    double total;
    double offset;
};

static std::vector<TrackingWheelData> get_lateral_data(const std::vector<TrackingWheel*>& sensors) {
    std::vector<TrackingWheelData> data;
    for (auto* sensor : sensors) {
        double distance = sensor->get_distance_delta();
        double total = sensor->get_distance_total();
        double offset = sensor->get_offset();
        data.push_back({distance, total, offset});
    } 
    return data;
}

static std::optional<double> calculate_wheel_heading(const std::vector<TrackingWheelData>& data) {
    if (data.size() < 2) return std::nullopt;
    double d_1 = data.at(0).total;
    double d_2 = data.at(1).total;
    double o_1 = data.at(0).offset;
    double o_2 = data.at(1).offset;
    if (std::abs(o_1-o_2) < 1e-8) return std::nullopt;
    return (d_1 - d_2) / (o_1 - o_2);
}

static std::optional<double> fuse_imus(const std::vector<pros::IMU*>& sensors) {
    if (sensors.empty()) return std::nullopt;
    
    double sum_sin = 0, sum_cos = 0;
    const double deg_to_rad = M_PI / 180.0;
    
    for (const auto* sensor : sensors) {
        double heading = sensor->get_heading() * deg_to_rad;
        sum_sin += std::sin(heading);
        sum_cos += std::cos(heading);
    }
    return std::atan2(sum_sin, sum_cos);
}

static std::optional<double> kalman_fuse_theta(std::vector<pros::IMU*>& imus, std::vector<TrackingWheelData>& wheel_data, double& p, double r, double q) {
    auto imu_heading = fuse_imus(imus);
    auto wheel_heading = calculate_wheel_heading(wheel_data);

    if (!imu_heading && wheel_heading) return wheel_heading;
    if (!wheel_heading && imu_heading) return imu_heading;
    if (!wheel_heading && !imu_heading) return std::nullopt;

    double k = p / (p + r / imus.size());
    double theta_error = wrap_angle(imu_heading.value() - wheel_heading.value());
    double theta_estimate = wheel_heading.value() + k * theta_error;

    p = (1 - k) * p + q;

    return wrap_angle(theta_estimate);
}

struct Delta2D {
    double dx;
    double dy;
};

static Delta2D kalman_fuse_translation (
    const std::vector<TrackingWheelData>& horizontals, 
    const std::vector<TrackingWheelData>& verticals, 
    double d_theta, double& p_x, double& p_y, double r, double q) 
{
    double dy_sum = 0, dx_sum = 0;
    int dy_count = 0, dx_count = 0;

    // Cache frequently used values to avoid repeated calculations
    const double abs_d_theta = std::abs(d_theta);
    const bool use_arc_calc = abs_d_theta >= 1e-8;
    double sin_half_theta = 0;
    double d_theta_inv = 0;
    
    if (use_arc_calc) {
        sin_half_theta = std::sin(d_theta * 0.5);
        d_theta_inv = 1.0 / d_theta;
    }

    for (const auto& wheel : horizontals) {
        double dy;
        if (use_arc_calc) {
            dy = 2 * sin_half_theta * (wheel.distance * d_theta_inv + wheel.offset);
        } else {
            dy = wheel.distance;
        }
        double k = p_y / (p_y + r);
        dy_sum += k * dy;
        dy_count++;
        p_y = (1 - k) * p_y + q;
    }
    
    for (const auto& wheel : verticals) {
        double dx;
        if (use_arc_calc) {
            dx = 2 * sin_half_theta * (wheel.distance * d_theta_inv + wheel.offset);
        } else {
            dx = wheel.distance;
        }
        double k = p_x / (p_x + r);
        dx_sum += k * dx;
        dx_count++;
        p_x = (1 - k) * p_x + q;
    }

    double dx = dx_count ? (dx_sum / dx_count) : 0;
    double dy = dy_count ? (dy_sum / dy_count) : 0;
    return {dx, dy};
}

static void update_global_pose(Pose& pose, const Delta2D& d_translation, double heading) {
    double avg_theta = pose.heading + (heading - pose.heading) * 0.5;
    // Cache sin/cos calculations
    double cos_avg = std::cos(avg_theta);
    double sin_avg = std::sin(avg_theta);
    
    double dx_global = cos_avg * d_translation.dx - sin_avg * d_translation.dy;
    double dy_global = sin_avg * d_translation.dx + cos_avg * d_translation.dy;
    
    pose.x += dx_global;
    pose.y += dy_global;
    pose.heading = heading;
}

Odometry::Odometry(std::vector<pros::IMU*> imus, 
    std::vector<TrackingWheel*> v_wheels,
    std::vector<TrackingWheel*> h_wheels,
    double p_x, double p_y, double p_theta,
    double r_translation, double r_heading, double q) : 
    imus(imus),
    v_wheels(v_wheels),
    h_wheels(h_wheels),
    p_x(p_x), p_y(p_y), p_theta(p_theta),
    r_translation(r_translation), r_heading(r_heading), q(q) {}

void Odometry::update(Pose& pose) {
    auto h_wheel_data = get_lateral_data(h_wheels);
    auto v_wheel_data = get_lateral_data(v_wheels);
    auto heading = kalman_fuse_theta(imus, h_wheel_data, p_theta, r_heading, q);

    if (!heading) return; // or handle error

    double d_theta = wrap_angle(heading.value() - pose.heading);
    auto d_translation = kalman_fuse_translation(h_wheel_data, v_wheel_data, d_theta, p_x, p_y, r_translation, q);

    update_global_pose(pose, d_translation, heading.value());
}