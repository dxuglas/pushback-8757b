#include "devices.h"
#include "../include/robot/chassis.h"
#include "pros/misc.h"

Chassis chassis({-12, -14, -17}, {18, 19, 20}, 0.1, 0.1);
pros::Motor indexer(10);
pros::Motor ccw_rollers(4);
pros::Motor cw_rollers(9);

pros::Controller master(pros::E_CONTROLLER_MASTER);
