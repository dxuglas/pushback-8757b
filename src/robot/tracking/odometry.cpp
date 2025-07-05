#include "odometry.h"
#include "pros/imu.hpp"
#include "tracking_wheel.h"
#include "utils/angle.h"
#include "utils/pose.h"
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

/**
 * @brief struct representing data from ma tracking wheel
 */
struct TrackingWheelData {
    double distance; // distance delta reported by the tracking wheel
    double total; // total distance reported by the tracking wheel
    double offset; // offset of tracking wheel from tracking center
};

/**
 * @brief Get data from a vector of tracking wheels
 * 
 * @param sensors vector of tracking wheel references
 * 
 * @return std::vector<TrackingWheelData> vector containing the tracking wheel 
 * data of each wheel
 */
static std::vector<TrackingWheelData> get_wheel_data(const std::vector<TrackingWheel*>& sensors) {
    std::vector<TrackingWheelData> data;
    for (auto* sensor : sensors) {
        double distance = sensor->get_distance_delta();
        double total = sensor->get_distance_total();
        double offset = sensor->get_offset();
        data.push_back({distance, total, offset});
    } 
    return data;
}

/**
 * @brief calculate the heading of the robot based on tracking wheel data
 *
 * @param data tracking wheel data of atleast two (2) wheels
 * 
 * @return std::nullopt not enough data to calculate heading
 * @return std::optional<double> predicted heading of the robot
 */
static std::optional<double> calculate_wheel_heading(const std::vector<TrackingWheelData>& data) {
    // check there are enough wheels
    if (data.size() < 2) return std::nullopt;
    double d_1 = data.at(0).total;
    double d_2 = data.at(1).total;
    double o_1 = data.at(0).offset;
    double o_2 = data.at(1).offset;
    // check the wheels have difference in offset to prevent zero division
    if (std::abs(o_1-o_2) < 1e-8) return std::nullopt; 
    // calculate heading using arc length formula
    return (d_1 - d_2) / (o_1 - o_2);
}

/**
 * @brief calculate the circular mean of a vector of IMUs
 *
 * @param sensors vector of IMU refrences
 *
 * @return std::nullopt no IMUs
 * @return std::optional<double> predicted heading of the robot
 */
static std::optional<double> fuse_imus(const std::vector<pros::IMU*>& sensors) {
    // check sensors contains atleast one (1) IMU
    if (sensors.empty()) return std::nullopt;
    // calculate circular mean of IMU headings
    double sum_sin = 0, sum_cos = 0;
    for (auto* sensor : sensors) {
        double heading = to_radians(sensor->get_heading());
        sum_sin += std::sin(heading);
        sum_cos += std::cos(heading);
    }
    return std::atan2(sum_sin, sum_cos);
}

/**
 * @brief calculate predicted heading of the robot 
 *
 * Uses kalman filtering to calculate a predicted heading based on data from
 * tracking wheels and IMUs.
 *
 * @param imus vector of IMU refrences
 * @param wheel_data vector of wheel data refrences
 * @param p reference to p-value (uncertainty) of heading data
 * @param r r-value (expected noise) of heading data
 * @param q q-value (expected system noise) of the kalman filter
 *
 * @return std::nullopt not enough data to predict heading
 * @return std::optional<double> predicted heading of the robot
 */
static std::optional<double> kalman_fuse_theta(std::vector<pros::IMU*>& imus, std::vector<TrackingWheelData>& wheel_data, double& p, double r, double q) {
    auto imu_heading = fuse_imus(imus);
    auto wheel_heading = calculate_wheel_heading(wheel_data);

    // if only one data source is avialable skip fusion
    if (!imu_heading && wheel_heading) return wheel_heading;
    if (!wheel_heading && imu_heading) return imu_heading;
    if (!wheel_heading && !imu_heading) return std::nullopt;

    // calculate kalman gain
    double k = p / (p + r / imus.size());
    double theta_error = wrap_angle(imu_heading.value() - wheel_heading.value());
    // calculate robots predicted heading
    double theta_estimate = wheel_heading.value() + k * theta_error;

    // update uncertainty of heading
    p = (1 - k) * p + q;

    return wrap_angle(theta_estimate);
}

/**
 * @brief struct representing a change in 2D translation
 */
struct Delta2D {
    double dx; // change in x translation
    double dy; // change in y translation
};

/**
 * @brief calculate predicted local translation of the robot
 *
 * Calculates the arc travelled by each tracking wheel over an infinitesimal
 * interval, and then uses kalman filtering to calculate a predicted local
 * (robot-centric), (dx,dy)
 *
 * @param h_wheels vector of horizontal tracking wheel data references
 * @param v_wheels vector of vertical tracking wheel data references
 * @param d_theta change in robots heading
 * @param p_x reference to p-value (uncertainty) of x-translation data
 * @param p_y reference to p-value (uncertainty) of y-translation data
 * @param r r-value (expected noise) of translation data
 * @param q q-value (expected system noise) of kalman filter
 * 
 * @return Delta2D predicted (dx,dy) of robot
 */
static Delta2D kalman_fuse_translation (
    const std::vector<TrackingWheelData>& h_wheels, 
    const std::vector<TrackingWheelData>& v_wheels, 
    double d_theta, double& p_x, double& p_y, double r, double q) 
{
    double dy_sum = 0, dx_sum = 0;
    int dy_count = 0, dx_count = 0;

    for (const auto& wheel : h_wheels) {
        // if the heading is 0, assume a linear path, otherwise calculate arc
        double dy = (std::abs(d_theta) < 1e-8) ? wheel.distance : 2 * std::sin(d_theta / 2) * ((wheel.distance / d_theta) + wheel.offset);
        // calculate kalman gain
        double k = p_y / (p_y + r);
        // add weighted dy to total
        dy_sum += k * dy;
        dy_count++;
        // update uncertainty of y-translation
        p_y = (1 - k) * p_y + q;
    }
    for (const auto& wheel : v_wheels) {
        // same as above but for x-translation
        double dx = (std::abs(d_theta) < 1e-8) ? wheel.distance : 2 * std::sin(d_theta / 2) * ((wheel.distance / d_theta) + wheel.offset);
        double k = p_x / (p_x + r);
        dx_sum += k * dx;
        dx_count++;
        p_x = (1 - k) * p_x + q;
    }

    // prevent 0 division error
    double dx = dx_count ? (dx_sum / dx_count) : 0;
    double dy = dy_count ? (dy_sum / dy_count) : 0;
    return {dx, dy};
}

/**
 * @brief updates the robots global position
 * 
 * Converts the local (robot-centric) (dx,dy) to a global (field-centric) change
 * and updates the global position
 * 
 * @param pose pose to update
 * @param d_translation local (dx,dy)
 * @param heading updated robot heading 
 */
static void update_global_pose(Pose& pose, const Delta2D& d_translation, double heading) {
    // calculates the average heading throughout the robots movement
    double avg_theta = pose.heading + (heading - pose.heading) / 2.0;

    // converts local (dx,dy) to global
    double dx_global = std::cos(avg_theta) * d_translation.dx - std::sin(avg_theta) * d_translation.dy;
    double dy_global = std::sin(avg_theta) * d_translation.dx + std::cos(avg_theta) * d_translation.dy;
    pose.x += dx_global;
    pose.y += dy_global;
    pose.heading = heading;
}

/**
 * @brief configure the sensors and filter values used for tracking
 *
 * @param imus vector of IMU pointers
 * @param v_wheels vector of vertical tracking wheels
 * @param h_wheels vector of horizontal tracking wheels
 * @param p_x uncertainty of x-translation data
 * @param p_y uncertainty of y-translation data
 * @param p_theta uncertainty of heading data
 * @param r_translation expected noise of translation data
 * @param r_heading expected noise of heading data
 * @param q expected proccess nosie
 */
void Odometry::configure(
    std::vector<pros::IMU*> imus, 
    std::vector<TrackingWheel*> v_wheels,
    std::vector<TrackingWheel*> h_wheels,
    double p_x, double p_y, double p_theta,
    double r_translation, double r_heading, double q) 
{
    this->imus = imus;
    this->v_wheels = v_wheels;
    this->h_wheels = h_wheels;
    this->p_x = p_x;
    this->p_y = p_y;
    this->p_theta = p_theta;
    this->r_translation = r_translation;
    this->r_heading = r_heading;
    this->q = q;
}

/**
 * @brief update the estimated pose of the robot
 *
 * This function should always be run in a dedicated task. Updates the estimated
 * pose of the robot based on sensor data every (delay) milliseconds
 * 
 * @param pose pose to update
 * @param delay update loop delay (millis)
 */
void Odometry::update(Pose& pose, uint32_t delay) {
    // get the current uptime of the robot
    uint32_t p_time = pros::millis();

    while (true) {
        // find how long its been since the loop last updated
        uint32_t time_now = pros::millis();
        uint32_t d_time = time_now - p_time;

        auto h_wheel_data = get_wheel_data(h_wheels);
        auto v_wheel_data = get_wheel_data(v_wheels);
        auto heading = kalman_fuse_theta(imus, h_wheel_data, p_theta, r_heading, q);

        // !TODO
        if (!heading) {
            // handle error (log or break the loop)
            break;
        }

        // calculate the robots local change in position
        double d_theta = wrap_angle(heading.value() - pose.heading);
        auto d_translation = kalman_fuse_translation(h_wheel_data, v_wheel_data, d_theta, p_x, p_y, r_translation, q);

        // convert local change in position to global scope
        update_global_pose(pose, d_translation, heading.value());

        // factor in execution time to ensure consistent loop delays
        if (d_time > delay) p_time = pros::millis();
        uint32_t dummy_p_time = p_time;
        pros::Task::delay_until(&dummy_p_time, delay);
        p_time = dummy_p_time;
    }
}