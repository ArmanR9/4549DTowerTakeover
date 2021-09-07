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
#include "intake.hpp"
#include "okapi/api.hpp"
#include "tasks.hpp"
#include "auto_drive.hpp"
#include <algorithm>
#include "angler.hpp"


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

std::shared_ptr<pros::Task> tilter_task_ctor(nullptr);
std::shared_ptr<pros::Task> lift_task_ctor(nullptr);

void initialize() {
	pros::delay(1000); // Delay to remove any garbage from the kernel and V5 devices during initialization

	// Initialize default brake modes of drive and subsystem motors. 
	brake_mode_init();

	// Initialize and run GUI
  	gui_init();
	gui();
	
	// Reset all sensor data and odometry position data.
	encoder360B.reset();
	encoder360L.reset();
	encoder360R.reset();
	pos.reset_pos();

	// Zero tilter position
	tilter_mtr.tare_position();

	// Initialize RTOS tasks for subsystems and odometry
	tilter_task_ctor = std::make_shared<pros::Task>(tilter::tilter_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "tilter task");
	lift_task_ctor = std::make_shared<pros::Task>(lift::lift_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "lift task");
	pros::Task odometry_task_ctor(tracking_update, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry task");
	pros::Task intake_task_ctor(intake::intake_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake task");

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
	
	// Opcontrol started, disable auto flag, enable opcontrol flag.
	lift::g_auton_flag = false;
	lift::g_opc_flag = true;

	// Construct the drivetrain objects 
	Analog_Control opc_analog;
	DriverProfile* opc_profile = new DriverProfile();

	opc_analog.set_scalar(12.5);
	opc_profile->setDrive(DriverProfile::Drive_State::TANK_DRIVE); // Set DriveProfile to TankDrive
	pros::Task controller_printing_task_ctor(controller_print, opc_profile, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "controller printing task");

	while (true) {

		if(master_controller.get_digital(DIGITAL_LEFT)){
		 opc_profile->toggle_switch('L');
		}

		if(master_controller.get_digital(DIGITAL_RIGHT)){
		 opc_profile->toggle_switch('R');
		}

    	joystick_drive(opc_profile, &opc_analog); // Pass in the DriverProfile object to handle the drivetrain controls

		if(master_controller.get_digital(DIGITAL_R1)){
			intake_set(127);
		}

		else if (master_controller.get_digital(DIGITAL_R2)){
			intake_set(-127);
		}

		else if(light_sensor.get_value() > light_sensor_threshold2 && lift::g_clearTray &&  pros::millis() < lift::g_timeout){
			lift::g_readyToLift = false;
			int ispeed = pow(light_sensor.get_value()*0.01, 1.25);
			ispeed = std::clamp(ispeed, -120, -100);

			intake_set(ispeed);
		}		

		else{
			lift::g_clearTray = false;
			lift::g_readyToLift = true;

			intake_set(0);
		}

		pros::delay(10);
	}

}
