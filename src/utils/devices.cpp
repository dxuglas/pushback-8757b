#include "devices.h"
#include "../include/robot/chassis.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "robot/tracking/tracking_wheel.h"

Chassis chassis({-12, -14, -19}, {6, 8, 20}, 0.1, 0.1);

pros::Rotation h_rotation(9);
TrackingWheel h_wheel(&h_rotation, 2.0, -1.75);

pros::Rotation v_rotation(-13);
TrackingWheel v_wheel(&v_rotation, 2.0, 1.75);

pros::IMU imu(17);

pros::Motor intake(16);
pros::Motor rollers(-11);
pros::Motor indexer(10);

pros::adi::Pneumatics gate('A', false); 
pros::adi::Pneumatics scraper('B', false);
pros::adi::Pneumatics descore('C', false);

pros::Optical color_sort(1);
pros::Distance rear_distance(5);

pros::Controller master(pros::E_CONTROLLER_MASTER);

std::string alliance = "blue";
