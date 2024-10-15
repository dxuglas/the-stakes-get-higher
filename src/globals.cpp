#include "main.h"

/////////////////
////VARIABLES////
/////////////////

/* Chassis Info */
const double gear_ratio = (5/3);
const double wheel_diameter = 3.18;
const double wheel_circ = 3.14159 * wheel_diameter;

/* Autonomous */
int selected_auton = 0;
int alliance = -1;

/////////////////
/////DEVICES/////
/////////////////

/* Sensors */
#define PORT_IMU 19
#define PORT_LIFT_POT 'A'
/* Pneumatics */
#define PORT_GOAL_CLAMP 'H'
#define PORT_INTAKE_RISER 'C'
/* Motors */
#define PORT_DRIVE_LEFT_FRONT -13
#define PORT_DRIVE_LEFT_MID -12
#define PORT_DRIVE_LEFT_BACK -11
#define PORT_DRIVE_RIGHT_FRONT 3
#define PORT_DRIVE_RIGHT_MID 2
#define PORT_DRIVE_RIGHT_BACK 1
#define PORT_INTAKE 20
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