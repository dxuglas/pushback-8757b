#ifndef CHASSIS_H
#define CHASSIS_H

#include "pros/motor_group.hpp"
#include <optional>

class Chassis {
public:
    Chassis(std::initializer_list<int8_t> left_drive_motor_ports, 
        std::initializer_list<int8_t> right_drive_motor_ports, 
        std::optional<float> left_joystick_y_deadzone, 
        std::optional<float> right_joystick_y_deadzone);

    void tank(float left_joystick_y_position, float right_joystick_y_position);

private:
    pros::MotorGroup l_motors;
    pros::MotorGroup r_motors;
    
    float l_deadzone;
    float r_deadzone;
};


#endif // CHASSIS_H