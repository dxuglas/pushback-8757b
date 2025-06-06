#include "devices.h"
#include "../include/robot/chassis.h"
#include "pros/misc.h"

Chassis chassis({1, 2, 3}, {4, 5, 6}, 0.1, 0.1);

pros::Controller master(pros::E_CONTROLLER_MASTER);
