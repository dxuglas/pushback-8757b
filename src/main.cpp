#include "main.h"
#include "../include/utils/devices.h"
#include "pros/misc.h"

void initialize() {
    pros::lcd::initialize();
    chassis.configure_odometry(
        {&left_h_wheel, &right_h_wheel}, 
        {}, 
        {&left_imu});
    pros::delay(200);
    pros::Task([&] {
        while (true) {
            auto p = chassis.get_pose();
            pros::lcd::print(0, "X: %f", p.x);
            pros::lcd::print(1, "Y: %f", p.y);
            pros::lcd::print(2, "Theta: %f", p.heading);
            pros::delay(10);
        }
    });
}

void competition_initialize() {}

void disabled() {}

void autonomous() {}

void opcontrol() {
    chassis.start_odometry();
    int countdown = 0;
	while (true) {
        chassis.tank(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) || master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            ccw_rollers.move(-127);
            cw_rollers.move(-127);
            indexer.move(-127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            cw_rollers.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            ccw_rollers.move(127);
            indexer.move(127);
        } else {
            ccw_rollers.move(0);
            cw_rollers.move(0);
            indexer.move(0);
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && countdown < 0) {
            match_load_ramp.toggle();
            countdown = 10;
        }

        countdown--;
		pros::delay(20);                               // Run for 20 ms then update
	}
}