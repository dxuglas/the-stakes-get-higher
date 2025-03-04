#ifndef GLOBALS_H
#define GLOBALS_H

/////////////////
////Variables////
/////////////////

/* Chassis Info */
extern const double gear_ratio, wheel_size, wheel_circ;

/////////////////
/////DEVICES/////
/////////////////

/* Controller*/
extern pros::Controller master;

/* Sensors */
extern pros::IMU imu;
extern pros::Distance intake_distance;

/* Pneumatics */
extern pros::adi::Pneumatics goal_clamp, sweeper;

/* Motors */
extern  pros::Motor drive_left_front, drive_left_mid, drive_left_back, 
        drive_right_front, drive_right_mid, drive_right_back,
        arm, intake_right, intake_left;

extern pros::MotorGroup drive_left, drive_right, drive, intake;

#endif