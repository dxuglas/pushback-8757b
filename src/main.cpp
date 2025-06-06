#include "main.h"
#include "../include/utils/devices.h"
#include "pros/misc.h"

void initialize() {}

void competition_initialize() {}

void disabled() {}

void autonomous() {}

void opcontrol() {
	while (true) {
        chassis.tank(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		pros::delay(20);                               // Run for 20 ms then update
	}
}