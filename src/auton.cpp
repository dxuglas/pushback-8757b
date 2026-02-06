#include "lemlib/asset.hpp"
#include "main.h"
#include "../include/utils/config.h"

ASSET(left_to_block_txt);
ASSET(left_to_loader_txt);
ASSET(left_in_loader_txt);
ASSET(left_score_txt);
ASSET(left_to_center_txt);
ASSET(left_away_center_txt);
ASSET(right_to_block_txt);
ASSET(right_to_loader_txt);
ASSET(right_to_center_txt);
ASSET(right_away_center_txt);
ASSET(right_in_loader_txt);
ASSET(right_score_txt);

// SKILLS ASSETS

ASSET(skills_to_loader_one_txt);
ASSET(skills_in_loader_one_txt);
ASSET(skills_to_loader_two_txt);
ASSET(skills_in_goal_one_txt);
ASSET(skills_in_loader_two_txt);

void intake_long() {
    intake.move(-127);
    rollers.move(-127);
    indexer.move(-127);
}

void intake_long_slow() {
    intake.move(-90);
    rollers.move(-127);
    indexer.move(-127);
}

void intake_middle() {
    intake.move(-127);
    rollers.move(-127);
    indexer.move(80);
}

void intake_stop() {
    intake.move(0);
    rollers.move(0);
    indexer.move(0);
}

void outake() {
    intake.move(90);
    rollers.move(70);
    indexer.move(-127);
}

void left_loader_long() {
    /*
    * Collects three blocks off the field, empties the loader and scores all
    * in the long goal. 
    */

    intake_long();
    chassis.setPose(62.669,-15.628,270);
    chassis.follow(left_to_block_txt, 12, 4000);
    pros::delay(2000);
    chassis.turnToHeading(125, 200);
    pros::delay(400);
    chassis.follow(left_to_loader_txt, 15, 6000);
    chassis.turnToHeading(94, 1000);
    scraper.toggle();
    pros::delay(200);
    intake_long_slow();
    pros::delay(700);
    chassis.follow(left_in_loader_txt, 15, 1000);
    pros::delay(2000);
    chassis.follow(left_score_txt, 15, 4000, false);
    scraper.toggle();
    pros::delay(2000);
    gate.toggle();
}

void right_loader_long() {
    /*
    * Collects three blocks off the field, empties the loader and scores all
    * in the long goal. 
    */

    intake_long();
    chassis.setPose(62.669,15.628,270);
    chassis.follow(right_to_block_txt, 12, 4000);
    pros::delay(2000);
    chassis.turnToHeading(35, 600);
    pros::delay(400);
    chassis.follow(right_to_loader_txt, 15, 7000);
    scraper.toggle();
    chassis.turnToHeading(90, 1000);
    pros::delay(200);
    intake_long_slow();
    pros::delay(700);
    chassis.follow(right_in_loader_txt, 15, 700);
    pros::delay(3000);
    chassis.follow(right_score_txt, 15, 4000, false);
    scraper.toggle();
    pros::delay(2000);
    gate.toggle();
}

void right_center_loader_long() {
    /*
    * Collects three blocks off the field, empties the loader and scores all
    * in the long goal. 
    */

    intake_long();
    chassis.setPose(62.669,15.628,270);
    chassis.follow(right_to_block_txt, 12, 4000);
    pros::delay(2000);
    chassis.turnToHeading(220, 600);
    pros::delay(400);
    chassis.follow(right_to_center_txt, 15, 7000);
    pros::delay(1000);
    outake();
    pros::delay(1500);
    intake_long_slow();
    chassis.follow(right_away_center_txt, 15, 7000, false);
    intake_long_slow();
    chassis.turnToHeading(35, 600);
    chassis.follow(right_to_loader_txt, 15, 7000);
    scraper.toggle();
    chassis.turnToHeading(93, 1000);
    pros::delay(900);
    chassis.follow(right_in_loader_txt, 15, 700);
    pros::delay(1400);
    chassis.follow(right_score_txt, 15, 4000, false);
    scraper.toggle();
    pros::delay(1000);
    gate.toggle();
}

void left_center_loader_long() {
    /*
    * Collects three blocks off the field, empties the loader and scores all
    * in the long goal. 
    */

    intake_middle();
    chassis.setPose(62.669,-15.628,270);
    chassis.follow(left_to_block_txt, 12, 4000);
    pros::delay(2000);
    chassis.turnToHeading(135, 600);
    pros::delay(400);
    chassis.follow(left_to_center_txt, 15, 7000, false);
    pros::delay(1000);
    gate.toggle();
    pros::delay(1500);
    gate.toggle();
    intake_long_slow();
    chassis.follow(left_away_center_txt, 15, 7000);
    chassis.follow(left_to_loader_txt, 15, 7000);
    scraper.toggle();
    chassis.turnToHeading(93, 1000);
    pros::delay(900);
    chassis.follow(left_in_loader_txt, 15, 1200);
    pros::delay(1600);
    chassis.follow(left_score_txt, 15, 4000, false);
    scraper.toggle();
    pros::delay(1000);
    gate.toggle();
}

void skills() {
    intake_long();
    chassis.setPose(-58.44, -16.312, 180);
    chassis.follow(skills_to_loader_one_txt, 12, 3000);
    scraper.toggle();
    pros::delay(300);
    chassis.turnToHeading(270, 1000);
    chassis.follow(skills_in_loader_one_txt, 12, 3000);
    pros::delay(1400);
    scraper.toggle();
    chassis.follow(skills_to_loader_two_txt, 15, 7000, false);
    chassis.turnToHeading(90, 1000);
    chassis.follow(skills_in_goal_one_txt, 12, 1000, false);
    intake_stop();
    pros::delay(300);
    gate.toggle();
    pros::delay(200);
    intake_long();
    pros::delay(3000);
    gate.toggle();
    chassis.setPose(29.445, -46.965, 90);
    scraper.toggle();
    chassis.follow(skills_in_loader_two_txt, 12, 1000);
}


void autonomous() {
    switch(selected_auton) {
        case 0:
            left_loader_long();
            break;
        case 1:
            left_center_loader_long();
            break;
        case 2: 
            right_loader_long();
            break;
        case 3:
            right_center_loader_long();
            break;
    }
}