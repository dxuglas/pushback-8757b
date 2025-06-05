#include "pros/motor_group.hpp"

class Chassis {
public:
    Chassis(std::initializer_list<int8_t> left_drive_motor_ports, std::initializer_list<int8_t> right_drive_motor_ports) 
        : l_motors(left_drive_motor_ports), r_motors(right_drive_motor_ports) {
    }

private:
    pros::MotorGroup l_motors;
    pros::MotorGroup r_motors;
};
