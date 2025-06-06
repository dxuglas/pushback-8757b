#include "chassis.h"
#include "../include/utils/check_threshold.h"

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