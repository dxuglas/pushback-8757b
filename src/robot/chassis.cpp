#include "pros/motor_group.hpp"

class Chassis {
public:
    Chassis(std::initializer_list<int8_t> left_drive_motor_ports, std::initializer_list<int8_t> right_drive_motor_ports) 
        : l_motors(left_drive_motor_ports), r_motors(right_drive_motor_ports) {
    }

    void tank(float left_joystick_y_position, float right_joystick_y_position) {
        l_motors.move(left_joystick_y_position);
        r_motors.move(right_joystick_y_position);
    }

private:
    pros::MotorGroup l_motors;
    pros::MotorGroup r_motors;
};
