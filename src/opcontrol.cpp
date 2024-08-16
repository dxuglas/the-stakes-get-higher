#include "main.h"

#define INTAKE_FORWARD pros::E_CONTROLLER_DIGITAL_L1
#define INTAKE_REVERSE pros::E_CONTROLLER_DIGITAL_L2
#define LIFT_UP pros::E_CONTROLLER_DIGITAL_R1
#define LIFT_DOWN pros::E_CONTROLLER_DIGITAL_R2

class Controls {
	public:
		void update() {
			update_drive();
			update_intake();
			update_lift();
		}
	private:
		int drive_direction = 1; 
		int left_joystick_dead_zone = 1;
		int right_joystick_dead_zone = 1;

		void update_drive() {
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
				// Set drive direction to inverse if button toggle is pressed
				drive_direction *= -1;
			}

			double left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
			double right_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

			if (left_y < left_joystick_dead_zone) left_y = 0; // Left dead zone
			if (right_y < right_joystick_dead_zone) right_y = 0; // Right dead zone
			
			drive_left.move(left_y * drive_direction * 127); // Left drive velocity
			drive_right.move(right_y * drive_direction * 127); // Right drive velocity
		}

		void update_intake() {
			if (master.get_digital(INTAKE_FORWARD)) {
				intake.move(127);
			} else if (master.get_digital(INTAKE_REVERSE)) {
				intake.move(-127);
			} else {
				intake.move(0);
			}
		}

		void update_lift() {
			if (master.get_digital(LIFT_UP)) {
				lift.move(127);
			} else if (master.get_digital(LIFT_DOWN)) {
				lift.move(-127);
			} else {
				lift.move(0);
			}
		}
};

void opcontrol() {
	Controls controls;
 	while (true) {
		controls.update();
		pros::delay(5);
	}
}