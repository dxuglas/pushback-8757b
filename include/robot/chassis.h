#ifndef CHASSIS_H
#define CHASSIS_H

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "tracking/odometry.h"
#include "tracking/tracking_wheel.h"
#include "utils/pose.h"
#include <cstdint>
#include <optional>

class Chassis {
public:
    // Constructors
    Chassis(std::initializer_list<int8_t> left_drive_motor_ports, 
        std::initializer_list<int8_t> right_drive_motor_ports, 
        std::optional<float> left_joystick_y_deadzone, 
        std::optional<float> right_joystick_y_deadzone);

    // User Control
    void tank(float left_joystick_y_position, float right_joystick_y_position);

    // Odometry
    void set_pose(float x, float y, float heading);
    void set_pose(Pose pose);
    Pose get_pose();
    void configure_odometry(
        std::vector<TrackingWheel*> h_wheels,
        std::vector<TrackingWheel*> v_wheels,
        std::vector<pros::IMU*> imus, 
        double p_x = 1.0, double p_y = 1.0, double p_theta = 0.0, 
        double r_translation = 1e-4, double r_heading = 1e-4, double q = 1e-4
    );
    void start_odometry(uint32_t delay = 10); 

private:
    // Devices
    pros::MotorGroup l_motors;
    pros::MotorGroup r_motors;
    
    // User Control
    float l_deadzone;
    float r_deadzone;

    // Odometry
    Pose pose = {0.0f, 0.0f, 0.0f};
    Odometry odometry;
    std::optional<pros::Task> odometry_task = std::nullopt;
};

#endif // CHASSIS_H