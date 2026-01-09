#include "main.h"
#include "../include/utils/devices.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "auton.h"


void initialize() {}

void competition_initialize() {}

void disabled() {}

void opcontrol() {
    disable();

    bool wrong_alliance_block = false;
    pros::Task color_sort_task([&wrong_alliance_block] {
        while(true) {
            int hue = color_sort.get_hue();
            if ((alliance == "red" && hue < 220 && hue > 140) || (alliance == "blue" && hue < 40 && hue > 0)) {
                wrong_alliance_block = true;
                pros::delay(20);
            } else {
                wrong_alliance_block = false;
            }
            pros::delay(10);
        }
    });


    pros::Task auto_gate_task([] {
        while (true) {
            if (gate.is_extended() && !master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
                while (rear_distance.get() < 150 ) {
                    pros::delay(20);
                }
                gate.retract();
            }
    }});

	while (true) {
        int wrong_alliance_sign = wrong_alliance_block ? 1 : -1;

        chassis.tank(master.get_analog(
                        pros::E_CONTROLLER_ANALOG_LEFT_Y), 
                    master.get_analog(
                        pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(-127);
            rollers.move(-127);
            indexer.move(127*wrong_alliance_sign);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(-127);
            indexer.move(-127*wrong_alliance_sign);
            rollers.move(-127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(127);
            indexer.move(127);
            rollers.move(127);
        } else {
            intake.move(0);
            indexer.move(0);
            rollers.move(0);
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            if (rear_distance.get() < 700) {
                gate.extend();
            }
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            gate.extend();
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            scraper.toggle();
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            descore.toggle();
        }

		pros::delay(20); // Run for 20 ms then update
	}
}
