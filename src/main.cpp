#include "main.h"
#include "../include/utils/devices.h"
#include "../include/utils/motor_control.h"
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
            set_motors_velocity(-127, ccw_rollers, cw_rollers, indexer);
        } else if (l1) {
            set_motors_velocity(127, ccw_rollers, cw_rollers, indexer);
        } else if (r1) {
            set_motors_velocity(127, ccw_rollers, cw_rollers);
            indexer.move(-127);
        } else {
            set_motors_velocity(0, ccw_rollers, cw_rollers, indexer);
        }

		pros::delay(20);                               // Run for 20 ms then update
	}
}