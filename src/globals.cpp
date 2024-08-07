#include "main.h"

// Sensors
#define PORT_IMU 1
#define PORT_LIFT_POT 'A'
// Pneumatics
#define PORT_GOAL_CLAMP 'B'
#define PORT_INTAKE_RISER 'C'
// Motors
#define PORT_DRIVE_LEFT_FRONT 8
#define PORT_DRIVE_LEFT_MID 9
#define PORT_DRIVE_LEFT_BACK 10
#define PORT_DRIVE_RIGHT_FRONT 18
#define PORT_DRIVE_RIGHT_MID 19
#define PORT_DRIVE_RIGHT_BACK 20
#define PORT_INTAKE 6
#define PORT_LIFT 7

/* Master */
pros::Controller master(pros::E_CONTROLLER_MASTER);

/* Sensors */
pros::IMU imu(PORT_IMU);
pros::adi::Potentiometer lift_pot(PORT_LIFT_POT);

/* Pneumatics */
pros::adi::Pneumatics goal_clamp(PORT_GOAL_CLAMP, false);
pros::adi::Pneumatics intake_riser(PORT_INTAKE_RISER, false);

/* Motors */
pros::Motor drive_left_front(PORT_DRIVE_LEFT_FRONT);
pros::Motor drive_left_mid(PORT_DRIVE_LEFT_MID);
pros::Motor drive_left_back(PORT_DRIVE_LEFT_BACK);
pros::Motor drive_right_front(PORT_DRIVE_RIGHT_FRONT);
pros::Motor drive_right_mid(PORT_DRIVE_RIGHT_MID);
pros::Motor drive_right_back(PORT_DRIVE_RIGHT_BACK);
pros::Motor intake(PORT_INTAKE);
pros::Motor lift(PORT_LIFT);

pros::MotorGroup drive_left({PORT_DRIVE_LEFT_FRONT, 
                             PORT_DRIVE_LEFT_MID,
                             PORT_DRIVE_LEFT_BACK});
pros::MotorGroup drive_right({PORT_DRIVE_RIGHT_FRONT, 
                             PORT_DRIVE_RIGHT_MID,
                             PORT_DRIVE_RIGHT_BACK});
pros::MotorGroup drive({PORT_DRIVE_LEFT_FRONT, 
                        PORT_DRIVE_LEFT_MID,
                        PORT_DRIVE_LEFT_BACK,
                        PORT_DRIVE_RIGHT_FRONT, 
                        PORT_DRIVE_RIGHT_MID,
                        PORT_DRIVE_RIGHT_BACK});