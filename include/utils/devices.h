#ifndef DEVICES_H
#define DEVICES_H

#include "pros/adi.hpp"
#include "robot/chassis.h"
#include "robot/tracking/tracking_wheel.h"

extern Chassis chassis;
extern TrackingWheel left_h_wheel, right_h_wheel, v_wheel;
extern pros::IMU left_imu;

extern pros::Motor indexer, ccw_rollers, cw_rollers;
extern pros::adi::Pneumatics match_load_ramp;
extern pros::Controller master;

#endif