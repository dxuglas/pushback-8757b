#ifndef DEVICES_H
#define DEVICES_H

#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "robot/chassis.h"
#include "robot/tracking/tracking_wheel.h"

extern Chassis chassis;
extern pros::Rotation left_h_rotation, right_h_rotation;
extern TrackingWheel left_h_wheel, right_h_wheel, v_wheel;
extern pros::IMU imu;

extern pros::adi::Pneumatics match_load_ramp;
extern pros::Distance block_detector, center_goal_sensor;
extern pros::Optical intake_sensor;
extern pros::adi::DigitalIn long_goal_sensor;
extern pros::adi::LineSensor left_line_sensor, right_line_sensor;
extern pros::adi::DigitalOut indexer_led_left, indexer_led_right;

extern pros::Motor indexer, intake_rollers, upper_stage_rollers, scoring_rollers;

extern pros::Controller master;

#endif