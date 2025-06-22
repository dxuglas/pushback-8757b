#include "chassis.h"
#include "../include/utils/check_threshold.h"
#include "utils/pose.h"
#include <cstdint>
#include <optional>
#include <sys/types.h>

Chassis::Chassis(std::initializer_list<int8_t> left_drive_motor_ports, 
                 std::initializer_list<int8_t> right_drive_motor_ports, 
                 std::optional<float> left_joystick_y_deadzone = std::nullopt, 
                 std::optional<float> right_joystick_y_deadzone = std::nullopt)
    : l_motors(left_drive_motor_ports), 
      r_motors(right_drive_motor_ports),
      l_deadzone(left_joystick_y_deadzone.value_or(0.0f)),
      r_deadzone(right_joystick_y_deadzone.value_or(0.0f)) {}

void Chassis::tank(float left_joystick_y_position, float right_joystick_y_position) {
    l_motors.move(check_threshold(left_joystick_y_position, l_deadzone));
    r_motors.move(check_threshold(right_joystick_y_position, r_deadzone));
}

void Chassis::set_pose(float x, float y, float heading) {
    Chassis::pose = {x, y, heading};
}

void Chassis::set_pose(Pose pose) {
    Chassis::pose = pose;
}

Pose Chassis::get_pose() {
    return pose;
}

void Chassis::configure_odometry(
        std::vector<TrackingWheel*> h_wheels,
        std::vector<TrackingWheel*> v_wheels,
        std::vector<pros::IMU*> imus, 
        double p_x, double p_y, double p_theta, 
        double r_translation, double r_heading, double q
    )
{
    odometry.configure(imus, v_wheels, h_wheels, p_x, p_y, p_theta, r_translation, r_heading, q);
}

void Chassis::start_odometry(uint32_t delay) {
    if (odometry_task == std::nullopt) {
        odometry_task = pros::Task([delay, this] { odometry.update(pose, delay);});
    }
}