#ifndef DEVICES_H
#define DEVICES_H

#include "robot/chassis.h"

extern Chassis chassis;
extern pros::Motor indexer, ccw_rollers, cw_rollers;
extern pros::Controller master;

#endif