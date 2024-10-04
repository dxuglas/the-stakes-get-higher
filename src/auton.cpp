#include "main.h"
#include <numeric>


void turn(double turnDegrees, int speedCap = 12000)
{

  double Kp = 95;
  double Ki = 1;
  double Kd = 15;

  double error = 0;
  double previousError = 0;
  double integral = 0;
  double derivative = 0;

  bool turnFinished = false;
  int turnCount = 0;

  imu.tare_rotation();
  imu.tare_rotation();

  while (!turnFinished)
  {

    double error = turnDegrees - imu.get_rotation();

    integral = integral + error;
    if (abs(error) < 30) {
      integral = 0;
    }

    derivative = error - previousError;
    if (derivative < 1 && abs(error) <= 20)
    {
      turnCount++;
      if (turnCount >= 20)
      {
        turnFinished = true;
      }
    }

    previousError = error;

    double speed = Kp * error + Ki * integral + Kd * derivative;

    if (speed > speedCap)
    {
      speed = speedCap;
    }
    if (speed < -speedCap)
    {
      speed = -speedCap;
    }

    drive_left.move_voltage(speed);
    drive_right.move_voltage(-speed);

    pros::Task::delay(10);
  }

  drive.move_voltage(0);
}

void move(double driveDistance, int speedCap = 12000)
{ 
  double distancePerDegree = (wheel_circ * gear_ratio) / 360;
  double distance = (driveDistance / distancePerDegree);

  double Kp = 10;
  double Ki = 0;
  double Kd = 0;

  double error = 0;
  double previousError = 0;

  double integral = 0;

  double derivative = 0;

  bool driveFinished = false;
  int exitTime = 300;
  int exitCount = 0;

  drive_left_mid.tare_position();
  drive_right_mid.tare_position();

  while (!driveFinished && exitTime > 0)
  {
    error = distance - (drive_left_mid.get_position() + drive_right_mid.get_position())/2;
    master.set_text(1,1,std::to_string(error));

    derivative = error - previousError;
    integral = integral + error;


    if (error < 200)
    {
      integral = 0;
    }

    double speed = Kp * error + Ki * integral + Kd * derivative;

    if (derivative > -3 && abs(error) <= 100)
    {
      exitCount = exitCount + 1;
      if (exitCount >= 25)
      {
        driveFinished = true;
      }
    }

    drive.move_voltage(speed);
    exitTime--;
    pros::Task::delay(10);
  }
  drive.move_voltage(0);
}

void auton_debug()
{
}

void red_line_goal()
{
  move(12);
}


void autonomous()
{
  // Negative (-) cases are connected to Red Alliance autons
  // Positive cases are connected to Blue Alliance autons
  // 0 is debug
  switch (selected_auton) {
    case -4:
      break;
    case -3:
      break;
    case -2:
      break;
    case -1:
      red_line_goal();
    case 0:
      auton_debug();
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