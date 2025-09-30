#include "main.h"
#include "utils/devices.h"

int selected_auton = 1;
int alliancce = 1;

class ChassisController
{
public:
  double drive_kp = 0;
  double drive_ki = 0;
  double drive_kd = 0;

  double turn_kp = 0;
  double turn_ki = 0;
  double turn_kd = 0;

  int integral_bound = 3; // Error boundary where integral comes into effect
  int integral_limit = 150; // Limit to prevent integral windup

  bool enabled = true;

  ChassisController(double drive_p, double drive_i, double drive_d,
                    double turn_p, double turn_i, double turn_d)
  { // Constructor for standard controller setup with dual PIDs
    drive_kp = drive_p;
    drive_ki = drive_i;
    drive_kd = drive_d;

    turn_kp = turn_p;
    turn_ki = turn_i;
    turn_kd = turn_d;
  }

  void update()
  { // Run chassis controller updates here
    int linear_velocity = linear_controller();
    int angular_velocity = angular_controller();

    if (drive_turn_toggle)
    {
      chassis.tank(linear_velocity, linear_velocity);
    }
    else
    {
      chassis.tank(angular_velocity, -angular_velocity);
    }
    time_exit--;
  }


  void turn_relative(double degrees)
  { // Turn relative to the current heading (in degrees)
    time_exit = 100;
    turn_reset = true;
    drive_turn_toggle = false;
    angle = degrees;
    turn_error = 2;
    while (abs(turn_error) > 1)
    {
      pros::Task::delay(1);
      if (time_exit < 0)
      {
        break;
      }
    }
  }

  void turn_absolute(double heading)
  { // Turn to an absolute heading (in degrees)
    time_exit = 200;
    double degrees;
    double difference = -chassis.get_pose().heading * 180 / M_PI - heading;
    turn_reset = true;
    drive_turn_toggle = false;

    if (difference < -180)
    {
      degrees = -(360 + difference);
    }
    else if (difference > 180)
    {
      degrees = 360 - difference;
    }
    else
    {
      degrees = -difference;
    }

    angle = degrees;
    turn_error = 2;
    while (abs(turn_error) > 1)
    {
      pros::Task::delay(1);
      if (time_exit < 0)
      {
        break;
      }
    }
  }

  void move(double distance_in_inches, double speed_percent = 100)
  { // Move the drive a set distance
    time_exit = 100;
    chassis.reset_position();
    drive_turn_toggle = true;
    distance = distance_in_inches;
    max_voltage_percent = speed_percent / 100;
    drive_error = 2;
    while (abs(drive_error) > 1)
    {
      pros::Task::delay(1);
      if (time_exit < 0)
      {
        break;
      }
    }
  }

  void enable()
  { // Enable the controller
    enabled = true;
  }

  void disable()
  { // Disabled the controller
    enabled = false;
  }

private:
  int drive_error = 0;
  int drive_last_error = 0;
  int drive_integral = 0;
  int drive_derivative = 0;
  double distance = 0;
  double max_voltage_percent = 1;
  double drive_zero;
  double drive_previous = 0;

  bool drive_reset = true;

  int turn_error = 0;
  int turn_last_error = 0;
  int turn_integral = 0;
  int turn_derivative = 0;
  int angle = 0;
  double turn_zero;

  bool turn_reset = true;
  int time_exit = 100;

  bool drive_turn_toggle = false;

  int sign_value(int value)
  { // Get sign (+/-) of value
    int sign = value < 0 ? -1 : 1;
    return sign;
  }

  int linear_controller()
  { // Linear PID Controller
    double drive_position = chassis.get_position();

    drive_error = distance - drive_position;
    drive_derivative = drive_error - drive_last_error;
    drive_last_error = drive_error;

    // Start integral accumulation once error is past integral bound, otherwise reset the integral
    drive_integral = abs(drive_error) < integral_bound ? drive_integral + drive_error : 0;

    // If integral is above it's limit, decrease it to limit. 
    drive_integral = abs(drive_integral) > integral_limit ? sign_value(drive_integral) * integral_limit : drive_integral;



    double output_voltage = std::clamp(int(drive_error * drive_kp + drive_integral * drive_ki + drive_derivative * drive_kd), -127, 127) * max_voltage_percent; 
    
    double max_delta = 5;
    
    if (std::abs(output_voltage - drive_previous) > max_delta) {
        if (output_voltage > drive_previous) {
            output_voltage = drive_previous + max_delta;
        } else{
            output_voltage = drive_previous - max_delta;
        }
    }
    drive_previous = output_voltage;
    printf("position %f zero %f error %d voltage %f\n", drive_position, drive_zero, drive_error, output_voltage);
    return output_voltage;
  }

  int angular_controller()
  { // Angular PID Controller
    if (turn_reset)
    { // Reset controller zero position on turn call
      turn_zero = -chassis.get_pose().heading * 180 / M_PI;
      turn_reset = false;
      pros::Task::delay(10);
    }

    turn_error = angle - -chassis.get_pose().heading * 180 / M_PI;
    turn_derivative = turn_error - turn_last_error;
    turn_last_error = turn_error;

    // Start integral accumulation once error is past integral bound, otherwise reset the integral
    turn_integral = abs(turn_error) < integral_bound ? turn_integral + turn_error : 0;

    // If integral is outside it's limit, decrease/increase it to limit. 
    turn_integral = abs(turn_integral) > integral_limit ? sign_value(turn_integral) * integral_limit : turn_integral;

    return turn_error * turn_kp + turn_integral * turn_ki + turn_derivative * turn_kd;
  }
};

// Primary chassis controller for autonomous functions
ChassisController controller(
  6.8, // Drive Kp 
  0.001, // Ki
  0.2, // Kd
  0.7, // Turn Kp
  0.004, // Ki
  0.04  // Kd
);

void chassis_task_loop(void* param)
{
  while (controller.enabled) {
    controller.update();
    pros::delay(10);
  }
}

void hold() {
    intake_rollers.move(0);
    indexer.move(0);
    upper_stage_rollers.move(0);
    scoring_rollers.move(-30);
}

void score_top() {
    intake_rollers.move(127);
    indexer.move(127);
    upper_stage_rollers.move(127);
    scoring_rollers.move(127);
}

void intake() {
    intake_rollers.move(127);
    indexer.move(127);
    upper_stage_rollers.move(24);
    scoring_rollers.move(-30);
}

void score_center() {
    intake_rollers.move(127);
    indexer.move(-127);
    if (block_detector.get() < 30) {
        upper_stage_rollers.move(-127);
    } else {
        upper_stage_rollers.move(127);
    }
    scoring_rollers.move(-127);
}

void score_bottom() {
    intake_rollers.move(-127);
    indexer.move(-127);
    upper_stage_rollers.move(-127);
}

void autonomous() {
    chassis.start_odometry();

    // intake();
    // chassis.tank(127, 127);
    // pros::delay(320);
    // chassis.tank(0, 0);
    // pros::delay(600);
    // chassis.tank(-60, 60);
    // pros::delay(250);
    // chassis.tank(0, 0);
    // pros::delay(600);
    // chassis.tank(40, 40);
    // pros::delay(1300);
    // chassis.tank(0, 0);
    // pros::delay(600);
    // chassis.tank(-40, -40);
    // pros::delay(525);
    // chassis.tank(0, 0);
    // pros::delay(600);
    // chassis.tank(60, -60);
    // pros::delay(385);
    // chassis.tank(0, 0);
    // match_load_ramp.toggle();
    // pros::delay(600);
    // chassis.tank(40, 40);
    // pros::delay(675);
    // chassis.tank(0, 0);
    // pros::delay(600);
    // score_center();
        
    intake();
    chassis.tank(127, 127);
    pros::delay(330);
    chassis.tank(0, 0);
    pros::delay(600);
    chassis.tank(60, -60);
    pros::delay(250);
    chassis.tank(0, 0);
    pros::delay(600);
    chassis.tank(40, 40);
    pros::delay(1300);
    chassis.tank(0, 0);
    pros::delay(600);
    chassis.tank(-40, -40);
    pros::delay(600);
    chassis.tank(0, 0);
    pros::delay(600);
    chassis.tank(-60, 60);
    pros::delay(340);
    chassis.tank(0, 0);
    pros::delay(600);
    chassis.tank(40, 40);
    pros::delay(650);
    chassis.tank(0, 0);
    pros::delay(600);
    score_bottom();
}
