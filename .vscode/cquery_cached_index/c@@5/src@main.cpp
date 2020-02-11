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
//pros::Task lift_t;
//pros::Task tilter_t;
std::shared_ptr<pros::Task> tilter_task_ctor(nullptr);
std::shared_ptr<pros::Task> lift_task_ctor(nullptr);

void initialize() {
	pros::delay(1000);
  gui_init();
	gui();
	encoder360B.reset();
	encoder360L.reset();
	encoder360R.reset();
	tilter_mtr.tare_position();
//	reset_sensors();
//	pros::lcd::initialize();

//	pros::lcd::set_text(0, "Init Auton Selector using btns");
//	pros::lcd::set_text(3, "Auton is 0");


//	pros::lcd::register_btn2_cb(auton_index_right);
//	pros::lcd::register_btn1_cb(auton_locker);
//	pros::lcd::register_btn0_cb(auton_index_left);
//pros::Task tilter_task_ctor(tilter::tilter_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "tilter task");

//	pros::Task odometry_task_ctor(tracking_update, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry task");
tilter_task_ctor = std::make_shared<pros::Task>(tilter::tilter_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "tilter task");
lift_task_ctor = std::make_shared<pros::Task>(lift::lift_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "lift task");

pros::Task odometry_task_ctor(tracking_update, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry task");
pros::Task intake_task_ctor(intake::intake_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake task");

//___int_least8_t_definedlift_t = &tilter_task_ctor;
//tilter_t = &tilter_task_ctor;

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

void autonomous(){
	//drive_set(50);
	turn2ang(90.0, 100, _TurnDir::CW, 10000, 5000);
	std::cout << "after AUTO " << radians_to_degrees(pos.get_alpha()) << std::endl << std::endl;
	//straight_line_drive(24,0,120,100,5000);
	/*
reset_mtr_encoders();
reset_encoders();
straight_line_drive(10,0,50,100,5000);

turn2ang(45, 50, 250, 2000);
switch(auton_sel){

    case(1):

        break;

    case(2):

        break;

    case(3):

        break;

    case(4):

        break;

    default:
       // straight_line_drive(12, 0, 100, 250, 2000);
        turn2ang(45, 50, 250, 2000);
        break;
      }
	  */
}


void opcontrol() {
	//lift_mtr.set_brake_mode(MOTOR_BRAKE_HOLD);
	bool pressed;
	lift::g_auton_flag = false;
	lift::g_opc_flag = true;
  int iterator{0};
	int iterator2{0};

	//okapi::Controller master2;

	okapi::ControllerButton r1(okapi::ControllerDigital::R1);
	okapi::ControllerButton r2(okapi::ControllerDigital::R2);
	//okapi::ControllerButton l1(okapi::ControllerDigital::L1);
	//okapi::ControllerButton l2(okapi::ControllerDigital::L2);

//	okapi::ControllerButton a(okapi::ControllerDigital::A);
	okapi::ControllerButton x(okapi::ControllerDigital::X);
	okapi::ControllerButton y(okapi::ControllerDigital::Y);
//	okapi::ControllerButton b(okapi::ControllerDigital::B);

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
/*
if(r1.changedToPressed() && iterator < lift::heights::E_NUM_OF_HEIGHTS - 1){
		++iterator;
		lift::setTarget(iterator);
	}

	else if(r2.changedToPressed() && iterator > 0){
		--iterator;
		lift::setTarget(iterator);
	}

*/
if(r1.isPressed()){
intake_set(127);
}

else if (r2.isPressed()){
intake_set(-127);
}

else { intake_set(0);}

if(master_controller.get_digital(DIGITAL_Y)){
drive_set(-35);
intake_set(-50);
}
/*
if(master_controller.get_digital(DIGITAL_L1)){
lift_mtr.move_velocity(75);
}
else if (master_controller.get_digital(DIGITAL_R2)){
lift_mtr.move_velocity(-50);
}
else { lift_mtr.move_velocity(0); }
*/

/*
if(a.changedToPressed() && iterator2 < 2){
iterator2++;
tilter::setTarget(tilter::State_Machine::E_STACK);
}

if(b.changedToPressed() && iterator2 > 0){
	iterator2 --;
tilter::setTarget(tilter::State_Machine::E_OFF);
}
*/

  //xyz.set_x(5);

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
	 //std::cout<<encoder360L.get() << "L " << encoder360R.get() << "R " << encoder360B.get() << "B " << std::endl;
		pros::delay(10);


	}
}
