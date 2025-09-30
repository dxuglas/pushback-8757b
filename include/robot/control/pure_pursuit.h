#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include "utils/pose.h"
#include "robot/chassis.h"
#include <vector>
#include <cstddef>
#include <optional>

/**
 * @brief Pure Pursuit controller with curvature-based speed control.
 */
class PurePursuit {
public:
    PurePursuit(Chassis& chassis, double lookahead_radius, double min_speed = 40, double max_speed = 127, double curvature_gain = 2.0);

    void set_path(const std::vector<Pose>& waypoints);

    /**
     * @brief Run one step of the pure pursuit algorithm.
     * @return true if path is complete, false otherwise.
     */
    bool step();

    void start(uint32_t delay_ms);

private:
    Chassis& chassis;
    std::vector<Pose> path;
    double lookahead_radius;
    std::size_t last_lookahead_idx;

    double min_speed;
    double max_speed;
    double curvature_gain;

    std::optional<Pose> get_lookahead(const Pose& robot_pose);
    double distance(const Pose& a, const Pose& b) const;
    double calculate_curvature(const Pose& robot_pose, const Pose& lookahead) const;
};

#endif // PURE_PURSUIT_H