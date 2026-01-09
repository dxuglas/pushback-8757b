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
      chassis.l_motors.move_voltage(linear_velocity);
      chassis.r_motors.move_voltage(0.93*linear_velocity);
    }
    else
    {
      chassis.l_motors.move_voltage(+angular_velocity);
      chassis.r_motors.move_voltage(-0.93*angular_velocity);
    }
    time_exit--;
  }


  void turn_relative(double degrees)
  { // Turn relative to the current heading (in degrees)
    time_exit = 250;
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
    time_exit = 150;
    double degrees;
    double difference = imu.get_heading() - heading;
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
    time_exit = 150;
    drive_reset = true;
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
    chassis.l_motors.move_voltage(0);
    chassis.r_motors.move_voltage(0);
  }

private:
  int drive_error = 0;
  int drive_last_error = 0;
  int drive_integral = 0;
  int drive_derivative = 0;
  double distance = 0;
  double max_voltage_percent = 1;
  double max_delta = 1;
  double previous_max_voltage = 0;

  bool drive_reset = true;

  int turn_error = 0;
  int turn_last_error = 0;
  int turn_integral = 0;
  int turn_derivative = 0;
  int angle = 0;

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
    if (drive_reset)
    { // Reset controller zero position on new move call
      chassis.l_motors.tare_position_all();
      chassis.r_motors.tare_position_all();
      drive_reset = false;
    }

    // Get average drive position from position vector and convert to inches from degrees
    double drive_average_position = (chassis.l_motors.get_position(0) + 
                                    chassis.l_motors.get_position(1) +
                                    chassis.l_motors.get_position(2) + 
                                    chassis.r_motors.get_position(0) +
                                    chassis.r_motors.get_position(1) +
                                    chassis.r_motors.get_position(2)) / 6;

    double drive_position_in_inches = (drive_average_position / 360) * 3.25 * M_PI * 3 / 5;

    drive_error = distance - drive_position_in_inches;
    drive_derivative = drive_error - drive_last_error;
    drive_last_error = drive_error;

    // Start integral accumulation once error is past integral bound, otherwise reset the integral
    drive_integral = abs(drive_error) < integral_bound ? drive_integral + drive_error : 0;

    // If integral is above it's limit, decrease it to limit. 
    drive_integral = abs(drive_integral) > integral_limit ? sign_value(drive_integral) * integral_limit : drive_integral;
    double target_voltage = drive_error * drive_kp + drive_integral * drive_ki + drive_derivative * drive_kd;
    target_voltage *= max_voltage_percent;

    double delta = target_voltage - previous_max_voltage;
    if (abs(delta) > max_delta) {
        // Clamp delta to max_delta
        target_voltage = previous_max_voltage + sign_value(delta) * max_delta;
    }
    previous_max_voltage = target_voltage;

    double output_voltage = drive_error * drive_kp + drive_integral * drive_ki + drive_derivative * drive_kd; 
    return output_voltage * max_voltage_percent;
  }

  int angular_controller()
  { // Angular PID Controller
    if (turn_reset)
    { // Reset controller zero position on turn call
      imu.tare_rotation();
      turn_reset = false;
      pros::Task::delay(10);
    }

    turn_error = angle - imu.get_rotation();
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
  700, // Drive Kp 
  5, // Ki
  12, // Kd
  92, // Turn Kp
  0.2, // Ki
  2.5  // Kd
);

void chassis_task_loop(void* param)
{
  while (controller.enabled) {
    controller.update();
    pros::delay(10);
  }
}

void disable() {
    controller.disable();
}

void intake_center() {
    intake.move(-127);
    rollers.move(-127);
    indexer.move(-127);
}

void intake_middle() {
    intake.move(-127);
    rollers.move(-127);
    indexer.move(127);
}

void intake_stop() {
    intake.move(0);
    rollers.move(0);
    indexer.move(0);
}

void outake() {
    intake.move(100);
    rollers.move(127);
    indexer.move(-127);
}

void right_long_goal() {
    intake_center();
    controller.move(18);
    controller.move(14, 60);
    pros::delay(400);
    controller.turn_relative(100);
    pros::delay(400);
    controller.move(25, 70);
    pros::delay(300);
    controller.turn_relative(60);
    scraper.toggle();
    controller.move(10, 100);
    controller.move(18, 100);
    pros::delay(500);
    controller.move(-40);
    gate.toggle();
}

void left_long_goal() {
    controller.move(18);
    intake_center();
    controller.move(12, 60);
    pros::delay(200);
    controller.turn_relative(-150);
    pros::delay(200);
    controller.move(24);
    pros::delay(300);
    controller.turn_relative(-60);
    scraper.toggle();
    pros::delay(400);
    controller.move(10, 100);
    controller.move(16, 100);
    pros::delay(500);
    controller.move(-40,80);
    gate.toggle();
}

void right_center_goal() {
    controller.move(18);
    intake_center();
    controller.move(23, 30);
    pros::delay(100);
    controller.turn_relative(-61);
    pros::delay(100);
    controller.move(16, 70);
    pros::delay(100);
    outake();
    pros::delay(1500);
    controller.move(-42);
    pros::delay(300);
    controller.turn_relative(-125);
    pros::delay(200);
    controller.move(-5);
    intake_center();
    scraper.toggle();
    pros::delay(400);
    controller.move(20, 100);
    pros::delay(500);
    controller.move(-40,80);
    gate.toggle();
}

void autonomous() {
    pros::Task chassis_task(chassis_task_loop, (void*)"Chasis" ,"Chassis"); 
    left_long_goal();
};