#include "main.h"
#include "lcd.hpp"
#include "motors.hpp"
#include "sensors.hpp"
#include "utilities.hpp"
#include "odometry.hpp"
#include "joystick.hpp"
#include "gui.hpp"
#include "controller_printing.hpp"
#include "tilter.hpp"
#include "lift.hpp"
#include "okapi/api.hpp"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::delay(1000);
  gui_init();
	gui();
//	reset_sensors();
//	pros::lcd::initialize();

//	pros::lcd::set_text(0, "Init Auton Selector using btns");
//	pros::lcd::set_text(3, "Auton is 0");


//	pros::lcd::register_btn2_cb(auton_index_right);
//	pros::lcd::register_btn1_cb(auton_locker);
//	pros::lcd::register_btn0_cb(auton_index_left);

	pros::Task odometry_task_ctor(tracking_update, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry task");
	pros::Task tilter_task_ctor(tilter::tilter_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "tilter task");
	pros::Task lift_task_ctor(lift::lift_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "lift task");

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	lift::g_auton_flag = false;
	lift::g_opc_flag = true;
	bool pressed = false;
	bool pressed2 = false;
  int iterator{0};

	okapi::Controller master2;
	okapi::ControllerButton r1(okapi::ControllerDigital::R1);
	okapi::ControllerButton r2(okapi::ControllerDigital::R2);

	Analog_Control opc_analog;
	DriverProfile* opc_profile = new DriverProfile();

	opc_profile->setDrive(DriverProfile::Drive_State::TANK_DRIVE);
	pros::Task controller_printing_task_ctor(controller_print, opc_profile, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "controller printing task");

	while (true) {

		if(master_controller.get_digital(DIGITAL_LEFT)){
		 opc_profile->toggle_switch('L');
		}

		if(master_controller.get_digital(DIGITAL_RIGHT)){
		 opc_profile->toggle_switch('R');
		}

    joystick_drive(opc_profile, &opc_analog); // pass in the drive profile opc_profile object


/*   if(master_controller.get_digital_new_press(DIGITAL_L1)){
			pressed = !pressed;

			if(pressed && iterator < heights::E_NUM_OF_HEIGHTS - 1){
					++iterator;
					lift::setTarget(iterator);
				}
}

 else if (master_controller.get_digital_new_press(DIGITAL_L2)){
pressed2 = !pressed2;

				if(pressed2 && iterator > heights::E_OFF){
				--iterator;
				lift::setTarget(iterator);
			}
}
*/

if(r1.changedToPressed() && iterator < lift::heights::E_NUM_OF_HEIGHTS - 1){
		++iterator;
		lift::setTarget(iterator);
	}

	else if(r2.changedToPressed() && iterator > 0){
		--iterator;
		lift::setTarget(iterator);
	}





		/*pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
										 */
	//	int left = master_controller.get_analog(ANALOG_LEFT_Y);
		//int right = master_controller.get_analog(ANALOG_RIGHT_Y);

		//LF_mtr = left;
		//RF_mtr = right;
		//std::cout << "hi\n";
	//	std::cout << master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) << '\n';
		pros::delay(10);


	}
}
