#include "main.h"
#include "../include/utils/devices.h"
#include "pros/misc.h"

void initialize() {}

void competition_initialize() {}

void disabled() {}

void autonomous() {}

void opcontrol() {
	while (true) {
        // Cache controller inputs to avoid multiple I/O calls
        float left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        float right_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        bool l1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool r1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        chassis.tank(left_y, right_y);

        if (l2 || r2) {
            ccw_rollers.move(-127);
            cw_rollers.move(-127);
            indexer.move(-127);
        } else if (l1) {
            ccw_rollers.move(127);
            cw_rollers.move(127);
            indexer.move(127);
        } else if (r1) {
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