#include "utils/config.h"

pros::MotorGroup left_drive({-12,-14,-19}, pros::MotorGearset::blue);
pros::MotorGroup right_drive({6,8,20}, pros::MotorGearset::blue);

lemlib::Drivetrain drive(&left_drive, &right_drive, 10.8, lemlib::Omniwheel::NEW_275, 360, 2);

pros::IMU imu(17);

pros::Rotation vertical(15);
pros::Rotation horizontal(-18);

lemlib::TrackingWheel vertical_track(&vertical, lemlib::Omniwheel::NEW_2, -0.75);
lemlib::TrackingWheel horizontal_track(&horizontal, lemlib::Omniwheel::NEW_2, 0);

lemlib::OdomSensors sensors(&vertical_track, nullptr, &horizontal_track, nullptr, &imu);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0.04, // integral gain (kI)
                                              4, // derivative gain (kD)
                                              2, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              3 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.3, // proportional gain (kP)
                                              0.05, // integral gain (kI)
                                              5.7, // derivative gain (kD)
                                              7, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drive, lateral_controller, angular_controller, sensors);

pros::Motor intake(16);
pros::Motor rollers(-13);
pros::Motor indexer(10);

pros::adi::Pneumatics gate('A', false); 
pros::adi::Pneumatics scraper('B', false);
pros::adi::Pneumatics descore('C', false);

pros::Optical color_sort(1);
pros::Distance rear_distance(5);

pros::Controller master(pros::E_CONTROLLER_MASTER);

int alliance = 1;
int selected_auton = 0;
