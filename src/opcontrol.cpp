#include "main.h"

// Controls
#define INTAKE_FORWARD pros::E_CONTROLLER_DIGITAL_R1
#define INTAKE_REVERSE pros::E_CONTROLLER_DIGITAL_R2
#define LIFT_UP pros::E_CONTROLLER_DIGITAL_L1
#define LIFT_DOWN pros::E_CONTROLLER_DIGITAL_L2
#define GOAL_CLAMP pros::E_CONTROLLER_DIGITAL_B
#define DRIVE_DIRECTION pros::E_CONTROLLER_DIGITAL_DOWN
#define DRIVE_LEFT pros::E_CONTROLLER_ANALOG_LEFT_Y
#define DRIVE_RIGHT pros::E_CONTROLLER_ANALOG_RIGHT_Y

class Controls {
	public:
		void update() {
			update_drive();
			update_intake();
			update_lift();
			update_clamp();
		}
	private:
		int drive_direction = 1; 
		int left_joystick_dead_zone = 10;
		int right_joystick_dead_zone = 10;

		void update_drive() {
			if (master.get_digital_new_press(DRIVE_DIRECTION)) {
				// Set drive direction to inverse if button toggle is pressed
				drive_direction *= -1;
				std::string controller_text = drive_direction > 0 ? "Drive: Forward" : "Drive: Reverse";
				master.set_text(0, 0, controller_text);
			}

			double left_y = master.get_analog(DRIVE_LEFT);
			double right_y = master.get_analog(DRIVE_RIGHT);

			if (abs(left_y) < left_joystick_dead_zone) left_y = 0; // Left dead zone
			if (abs(right_y) < right_joystick_dead_zone) right_y = 0; // Right dead zone
			
			drive_left.move(left_y * drive_direction); // Left drive velocity
			drive_right.move(right_y * drive_direction); // Right drive velocity
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

		void update_clamp () {
			if (master.get_digital_new_press(GOAL_CLAMP)) {
				goal_clamp.toggle();
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