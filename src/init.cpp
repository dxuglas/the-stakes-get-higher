#include "main.h"
#include "ui.h"

void initialize() {
  ui_init();
  imu.reset();

  drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void competition_initialize() {}

void disabled() {}


