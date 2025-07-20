#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "pros/motors.hpp"

// Helper function to set multiple motors simultaneously for better performance
inline void set_motors_velocity(int velocity, pros::Motor& motor1, pros::Motor& motor2, pros::Motor& motor3) {
    motor1.move(velocity);
    motor2.move(velocity);
    motor3.move(velocity);
}

// Overload for two motors
inline void set_motors_velocity(int velocity, pros::Motor& motor1, pros::Motor& motor2) {
    motor1.move(velocity);
    motor2.move(velocity);
}

#endif // MOTOR_CONTROL_H