#include "devices.h"
#include "../include/robot/chassis.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "robot/tracking/tracking_wheel.h"

Chassis chassis({-12, -14, -17}, {18, 19, 20}, 0.1, 0.1);

pros::Rotation left_h_rotation(16);
TrackingWheel left_h_wheel(&left_h_rotation, 2.0, -1.75);

pros::Rotation right_h_rotation(-15);
TrackingWheel right_h_wheel(&right_h_rotation, 2.0, 1.75);

pros::Rotation v_rotation(3);
TrackingWheel v_wheel(&v_rotation, 2.0, 1);

pros::IMU left_imu(11);

pros::Motor indexer(8);
pros::Motor ccw_rollers(4);
pros::Motor cw_rollers(9);

pros::adi::Pneumatics match_load_ramp('A', false);

pros::Controller master(pros::E_CONTROLLER_MASTER);
