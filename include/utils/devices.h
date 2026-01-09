#ifndef DEVICES_H
#define DEVICES_H

#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "robot/chassis.h"
#include "robot/tracking/tracking_wheel.h"

extern Chassis chassis;
extern pros::Rotation h_rotation, v_rotation;
extern TrackingWheel h_wheel, v_wheel;
extern pros::IMU imu;

extern pros::Motor indexer, intake, rollers;

extern pros::adi::Pneumatics gate, scraper, descore;

extern pros::Optical color_sort;

extern pros::Distance rear_distance;

extern pros::Controller master;

extern std::string alliance;

#endif