#include "main.h"
#include "../include/utils/devices.h"
#include "pros/misc.h"

void initialize() {}

void competition_initialize() {}

void disabled() {}

void autonomous() {}

void opcontrol() {
	while (true) {
        chassis.tank(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) || master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            ccw_rollers.move(-127);
            cw_rollers.move(-127);
            indexer.move(-127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            ccw_rollers.move(127);
            cw_rollers.move(127);
            indexer.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            ccw_rollers.move(127);
            cw_rollers.move(127);
            indexer.move(-127);
        } else {
            ccw_rollers.move(0);
            cw_rollers.move(0);
            indexer.move(0);
        }

		pros::delay(20);                               // Run for 20 ms then update
	}
}