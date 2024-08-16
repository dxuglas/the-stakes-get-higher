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

  int integral_bound = 3;
  int integral_limit = 200;

  bool enabled = true;

  ChassisController(double drive_p, double drive_i, double drive_d,
  double turn_p, double turn_i, double turn_d) 
  {
    drive_kp = drive_p;
    drive_ki = drive_i;
    drive_kd = drive_d;

    turn_kp = turn_p;
    turn_ki = turn_i;
    turn_kd = turn_d;
  }

  void start()
  {
    pros::Task chassis_task(trampoline, this, "Chassis");
  }

  void turn(double degrees) 
  {
    turn_reset = true;
    angle = degrees;
  }

  void move(double distance_) 
  {
    drive_reset = true;
    distance = distance_;
  }

  void enable()
  {
    enabled = true;
  }

  void disable()
  {
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

  static void trampoline(void* iparam)
  {
    if (iparam) 
    {
      ChassisController* that = static_cast<ChassisController*>(iparam);
      that->update();
      pros::Task::delay(10);
    }
  }

  void update()
  {
    while (enabled) 
    { 
      // Run chassis controller updates here
      int linear_velocity = linear_controller();
      int angular_velocity = angular_controller();
    
      drive_left.move(linear_velocity - angular_velocity);
      drive_right.move(linear_velocity + angular_velocity);
    }
  }

  int sign_value(int value) {
    int sign = value < 0 ? -1 : 1;
    return sign;
  }

  int linear_controller () 
  {
    if (drive_reset)
    {
      drive.tare_position_all();
      drive_reset = false;
    }     

    std::vector<double> drive_position = drive.get_position_all();
    double drive_average_position = std::accumulate(drive_position.begin(), 
    drive_position.end(), 0LL) / drive_position.size();

    double drive_position_in_inches = (wheel_circ / 360) * drive_average_position;

    drive_error = drive_position_in_inches - distance;
    drive_derivative = drive_error - drive_last_error;
    drive_last_error = drive_error;
    
    if (abs(drive_error) < integral_bound)
    {
      drive_integral += drive_error;
    } 
    else 
    {
      drive_integral = 0;
    }

    drive_integral = abs(drive_integral) > integral_limit ? sign_value(drive_integral) * integral_limit : drive_integral;

    return drive_error * drive_kp + drive_integral * drive_ki + drive_derivative * drive_kd;
  }

  int angular_controller() 
  {
    if (turn_reset)
    {
      imu.tare_rotation();
      turn_reset = false;
    }

    turn_error = imu.get_rotation() - angle;
    turn_derivative = turn_error - turn_last_error;
    turn_last_error = turn_error;

    if (abs(turn_error) < integral_bound)
    {
      turn_integral += turn_error;
    } 
    else 
    {
      turn_integral = 0;
    }

    turn_integral = abs(turn_integral) > integral_limit ? sign_value(turn_integral) * integral_limit : turn_integral;

    return turn_error * turn_kp + turn_integral * turn_ki + turn_derivative * turn_kd;
  }
};

ChassisController chassis(0, 0, 0, 0, 0, 0);

void autonomous()
{
  chassis.start();
}