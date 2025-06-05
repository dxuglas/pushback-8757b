#include "pros/motor_group.hpp"

class Chassis {
public:
    Chassis(std::initializer_list<int8_t> left_drive_motor_ports, std::initializer_list<int8_t> right_drive_motor_ports, 
        float left_joystick_y_deadzone, float right_joystick_y_deadzone) 
        : l_motors(left_drive_motor_ports), r_motors(right_drive_motor_ports) {
        l_deadzone = left_joystick_y_deadzone;
        r_deadzone = right_joystick_y_deadzone;    
    }

    void tank(float left_joystick_y_position, float right_joystick_y_position) {
        l_motors.move(check_deadzone(left_joystick_y_position, l_deadzone));
        r_motors.move(check_deadzone(right_joystick_y_position, r_deadzone));
    }

private:
    pros::MotorGroup l_motors;
    pros::MotorGroup r_motors;

    float l_deadzone;
    float r_deadzone;

    float check_deadzone(float stick_value, float min_value) {
        return stick_value > min_value ? stick_value : 0.0f;
    }

};
