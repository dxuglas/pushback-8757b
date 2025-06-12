#ifndef CHASSIS_H
#define CHASSIS_H

#include "pros/motor_group.hpp"
#include "utils/pose.h"
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

    

private:
    // Devices
    pros::MotorGroup l_motors;
    pros::MotorGroup r_motors;
    
    // User Control
    float l_deadzone;
    float r_deadzone;

    // Odometry
    Pose pose = {0.0f, 0.0f, 0.0f};
};

#endif // CHASSIS_H