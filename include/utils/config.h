#ifndef CONFIG_H
#define CONFIG_H

#include "main.h"
#include "pros/motor_group.hpp"

extern lemlib::Chassis chassis;
extern pros::MotorGroup left_drive, right_drive;

extern pros::IMU imu;

extern pros::Motor indexer, intake, rollers;

extern pros::adi::Pneumatics gate, scraper, descore;

extern pros::Optical color_sort;

extern pros::Distance rear_distance;

extern pros::Controller master;

extern int alliance, selected_auton;

#endif