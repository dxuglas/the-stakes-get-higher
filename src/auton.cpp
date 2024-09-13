#include "main.h"
#include <numeric>

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
  int integral_limit = 200; // Limit to prevent integral windup

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

  void start()
  { // Intialise the Controller Task
    pros::Task chassis_task(trampoline, this, "Chassis");
  }

  void turn_relative(double degrees)
  { // Turn relative to the current heading (in degrees)
    turn_reset = true;
    angle = degrees;
  }

  void turn_absolute(double heading)
  { // Turn to an absolute heading (in degrees)
    double degrees;
    double difference = imu.get_heading() - heading;

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
  }

  void move(double distance_)
  { // Move the drive a set distance
    drive_reset = true;
    distance = distance_;
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
  int drive_error;
  int drive_last_error = 0;
  int drive_integral = 0;
  int drive_derivative;
  double distance;

  bool drive_reset = true;

  int turn_error;
  int turn_last_error = 0;
  int turn_integral = 0;
  int turn_derivative = 0;
  int angle;

  bool turn_reset = true;

  static void trampoline(void *iparam)
  { // Wrapper of update method for task
    if (iparam)
    {
      ChassisController *that = static_cast<ChassisController *>(iparam);
      that->update();
      pros::Task::delay(10);
    }
  }

  void update()
  { // Main task loop for controller updates
    while (enabled)
    {
      // Run chassis controller updates here
      int linear_velocity = linear_controller();
      int angular_velocity = angular_controller();

      drive_left.move(linear_velocity - angular_velocity);
      drive_right.move(linear_velocity + angular_velocity);
    }
  }

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
    std::vector<double> drive_position = drive.get_position_all();
    double drive_average_position = std::accumulate(drive_position.begin(),
                                                    drive_position.end(), 0LL) /
                                    drive_position.size();

    double drive_position_in_inches = (wheel_circ / 360) * drive_average_position;

    drive_error = drive_position_in_inches - distance;
    drive_derivative = drive_error - drive_last_error;
    drive_last_error = drive_error;

    if (abs(drive_error) < integral_bound)
    { // Start integral accumulation once error is past integral bound
      drive_integral += drive_error;
    }
    else
    { // Otherwise reset the integral
      drive_integral = 0;
    }

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
    }

    turn_error = imu.get_rotation() - angle;
    turn_derivative = turn_error - turn_last_error;
    turn_last_error = turn_error;

    if (abs(turn_error) < integral_bound)
    { // Start integral accumulation once error is past integral bound
      turn_integral += turn_error;
    }
    else
    { // Otherwise reset the integral
      turn_integral = 0;
    }

    // If integral is above it's limit, decrease it to limit. 
    turn_integral = abs(turn_integral) > integral_limit ? sign_value(turn_integral) * integral_limit : turn_integral;

    return turn_error * turn_kp + turn_integral * turn_ki + turn_derivative * turn_kd;
  }
};



// Primary chassis controller for autonomous functions
ChassisController chassis(0, 0, 0, 0, 0, 0);

void autonomous()
{
  chassis.start(); // Start chassis task

  switch (selected_auton) {
    case -4:
      break;
    case -3:
      break;
    case -2:
      break;
    case -1:
      break;
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      break;
  }
}