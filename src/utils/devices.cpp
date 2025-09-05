#include "devices.h"
#include "../include/robot/chassis.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "robot/tracking/tracking_wheel.h"

Chassis chassis({-12, -14, -13}, {18, 19, 20}, 0.1, 0.1);

pros::Rotation left_h_rotation(16);
TrackingWheel left_h_wheel(&left_h_rotation, 2.0, -1.75);

pros::Rotation right_h_rotation(-15);
TrackingWheel right_h_wheel(&right_h_rotation, 2.0, 1.75);

pros::IMU imu(11);

pros::Distance block_detector(1);
pros::Optical intake_sensor(2);
pros::Distance center_goal_sensor(3);

pros::Motor indexer(10);
pros::Motor intake_rollers(4);
pros::Motor upper_stage_rollers(5);
pros::Motor scoring_rollers(6);

pros::adi::Pneumatics match_load_ramp('A', false);

pros::adi::DigitalOut indexer_led_left('B');
pros::adi::DigitalOut indexer_led_right('C');

pros::adi::DigitalIn long_goal_sensor('D');

pros::adi::LineSensor left_line_sensor('F');
pros::adi::LineSensor right_line_sensor('E');

pros::Controller master(pros::E_CONTROLLER_MASTER);
