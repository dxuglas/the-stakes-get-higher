#include "main.h"
#include <numeric>

extern int selected_auton;
extern int alliance;

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
      drive_left.move_voltage(linear_velocity);
      drive_right.move_voltage(linear_velocity);
    }
    else
    {
      drive_left.move_voltage(+angular_velocity);
      drive_right.move_voltage(-angular_velocity);
    }
  }


  void turn_relative(double degrees)
  { // Turn relative to the current heading (in degrees)
    turn_reset = true;
    drive_turn_toggle = false;
    angle = degrees;
    turn_error = 2;
    while (abs(turn_error) > 1)
    {
      pros::Task::delay(1);
    }
  }

  void turn_absolute(double heading)
  { // Turn to an absolute heading (in degrees)
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
    }
  }

  void move(double distance_in_inches)
  { // Move the drive a set distance
    drive_reset = true;
    drive_turn_toggle = true;
    distance = distance_in_inches;
    drive_error = 2;
    while (abs(drive_error) > 1)
    {
      pros::Task::delay(1);
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

  bool drive_reset = true;

  int turn_error = 0;
  int turn_last_error = 0;
  int turn_integral = 0;
  int turn_derivative = 0;
  int angle = 0;

  bool turn_reset = true;

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
      drive.tare_position_all();
      drive_reset = false;
    }

    // Get average drive position from position vector and convert to inches from degrees
    double drive_average_position = (drive_left_back.get_position() + 
                                    drive_left_mid.get_position() +
                                    drive_right_back.get_position() +
                                    drive_right_mid.get_position()) / 4;

    double drive_position_in_inches = (drive_average_position / 600) * wheel_circ;

    drive_error = distance - drive_position_in_inches;
    drive_derivative = drive_error - drive_last_error;
    drive_last_error = drive_error;

    // Start integral accumulation once error is past integral bound, otherwise reset the integral
    drive_integral = abs(drive_error) < integral_bound ? drive_integral + drive_error : 0;

    // If integral is above it's limit, decrease it to limit. 
    drive_integral = abs(drive_integral) > integral_limit ? sign_value(drive_integral) * integral_limit : drive_integral;

    return drive_error * drive_kp + drive_integral * drive_ki + drive_derivative * drive_kd;
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
ChassisController chassis(
  1084, // Drive Kp 
  9, // Ki
  69, // Kd
  95, // Turn Kp
  0.2, // Ki
  2.5  // Kd
);

void auton_debug()
{

}

void red_right_close_goal()
{
  chassis.move(-20);
  pros::delay(300);
  goal_clamp.toggle();
  pros::delay(500);
  chassis.turn_relative(-75);
  intake.move(127);
  chassis.move(10);
  chassis.move(2);
  pros::delay(3000);
  chassis.turn_relative(160);
  pros::delay(200);
  chassis.move(24);
}

void red_left_close_goal()
{
  chassis.move(-20);
  pros::delay(300);
  goal_clamp.toggle();
  pros::delay(500);
  chassis.turn_relative(75);
  intake.move(127);
  chassis.move(10);
  chassis.move(2);
  pros::delay(3000);
  chassis.turn_relative(160);
  pros::delay(200);
  chassis.move(24);
  // pros::delay(2000);
  // chassis.turn_relative(-157);
  // pros::delay(500);
  // chassis.move(30);
  // chassis.move(10);
  // pros::delay(1200);
  // intake.move(-127);
  // pros::delay(1000);
  // intake.move(127);
  // chassis.move(12);
}

void red_line_goal()
{
  chassis.move(-38);
  pros::delay(200);
  goal_clamp.toggle();  
  pros::delay(1000);
  // chassis.move(34);
  // pros::delay(700);
  // intake.move(127);
}

void blue_left_close_goal()
{
  chassis.move(-20);
  pros::delay(300);
  goal_clamp.toggle();
  pros::delay(500);
  chassis.turn_relative(75);
  intake.move(127);
  chassis.move(10);
  chassis.move(2);
  pros::delay(3000);
  chassis.turn_relative(160);
  pros::delay(200);
  chassis.move(24);
  
}

void blue_right_close_goal()
{
  chassis.move(-20);
  pros::delay(300);
  goal_clamp.toggle();
  pros::delay(500);
  chassis.turn_relative(-75);
  intake.move(127);
  chassis.move(10);
  chassis.move(2);
  pros::delay(3000);
  chassis.turn_relative(160);
  pros::delay(200);
  chassis.move(24);
}

void chassis_task_loop(void* param)
{
  while (chassis.enabled) {
    chassis.update();
    pros::delay(10);
  }
}

void autonomous()
{
  pros::Task chassis_task(chassis_task_loop, (void*)"Chasis" ,"Chassis");

  selected_auton *= alliance;

  // Negative (-) cases are connected to Red Alliance autons
  // Positive cases are connected to Blue Alliance autons
  // 0 is debug
  switch (selected_auton) {
    case -4:
      break;
    case -3:
      master.set_text(0,0,"redrightclose");
      red_right_close_goal();
      break;
    case -2:
      master.set_text(0,0,"redleftclose");
      red_left_close_goal();
      break;
    case -1:
      master.set_text(0,0,"redline");
      red_line_goal();
      break;
    case 0:
      auton_debug();
    case 1:
      break;
    case 2:
      master.set_text(0,0,"leftclose");
      blue_left_close_goal();
      break;
    case 3:
      blue_right_close_goal();
      break;
    case 4:
      break;
  }
}