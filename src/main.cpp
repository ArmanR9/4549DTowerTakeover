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
	brake_mode_init();
  gui_init();
	gui();
	encoder360B.reset();
	encoder360L.reset();
	encoder360R.reset();
	pos.reset_pos();

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
	bool readyToStack = false;
	using namespace okapi;
	using namespace okapi::literals;
<<<<<<< HEAD
//	pos.reset_pos();
=======
>>>>>>> 92fd986464b0fdbbc43183bc402b23742a241a64
	//	intake::set_targetAsync(intake::States::E_INTAKE, 15000);
//		pros::delay(10000);
	//	tilter::setTarget(tilter::State_Machine::E_LIFT);
	//	pros::delay(500);
	//	lift::setTargetAutonAsync(lift::heightsAUTO::E_DEPLOY, 700);
	//	pros::delay(700);
//		tilter::setTarget(tilter::State_Machine::E_OFF);
	//	pros::delay(500);
	//	lift::setTargetAutonAsync(lift::heightsAUTO::E_OFF, 700);

	//	driveToPosition(12.0, 0.0, pos.get_y(), pos.get_x(), 0.1, 100, true, true, false);
	//	position_sweep(6.0, 6.0, pos.get_y(), pos.get_x(), trueS);

//
 	switch(auton_sel){

					case(1): // Blue
					tilter::setTarget(tilter::State_Machine::E_LIFT);
				//	pros::delay(10);
<<<<<<< HEAD
				lift::setTargetAutonAsync(lift::heightsAUTO::E_MED, 1200);
			//	pros::delay(200);
				//intake::set_targetAsync(intake::States::E_OUTTAKE, 300);
				pros::delay(1200);
				tilter::setTarget(tilter::State_Machine::E_OFF);


					pros::delay(1600);
					intake::set_targetAsync(intake::States::E_INTAKE, 10000);
					driveToPosition(29.25, 0.0, 0.0, 0.0, 0.25, 90, 1800, true, true, false);
					driveToPosition(7.50, -23.0, 29.25, 0.0, 0.5, 90, 2000, true, false, false);
					turn2ang(0.0, 90, _TurnDir::CW, 500, 300);
					driveToPosition(48.0, -24.0, 7.50, -23.0, 0.5, 75, 2050, true, true, false);
					driveToPosition(25.0, -24.0, 48.0, -24.0, 0.5, 90, 1100, true, false, false);
					turn2ang(120.0, 75, _TurnDir::CCW, 500, 1050);

					driveToPosition(0.5, -48.5, 25.0, -24.0, 0.5, 100, 1000, true, true, false);
				//	driveToDistance(22.5, pos.get_alpha(), 25.0, -24.0, 0.5, 100, 1050, true, true, false);
					//straight_line_drive(22.0,, float kP_correction, float maxErrA, int max_velocity, uint32_t settle, uint32_t max_time)
				//	float d, float a, float ys, float xs, float maxErrX, float maxVel, std::uint32_t ifailsafe, bool enableCorrect, bool forward, bool harshStop
				//	intake::light_sen(285);
					intake::set_targetAsync(intake::States::E_OUTTAKE, 350);
					tilter::setTarget(tilter::State_Machine::E_STACK);
					pros::delay(2800);
=======
					lift::setTargetAutonAsync(lift::heightsAUTO::E_LOW, 750);
					pros::delay(200);
					intake::set_targetAsync(intake::States::E_OUTTAKE, 300);
					pros::delay(675);
					tilter::setTarget(tilter::State_Machine::E_OFF);
					pros::delay(1700);
					intake::set_targetAsync(intake::States::E_INTAKE, 10000);
			    driveToPosition(29.25, 0.0, 0.0, 0.0, 0.25, 90, 1800, true, true, false);
			    driveToPosition(7.50, -25.0, 29.25, 0.0, 0.5, 90, 2000, true, false, false);
					turn2ang(0.0, 90, _TurnDir::CW, 500, 300);
					driveToPosition(50.0, -25.0, 7.50, -25.0, 0.5, 75, 2050, true, true, false);
					driveToPosition(25.0, -25.0, 50.0, -25.0, 0.5, 90, 1100, true, false, false);
					turn2ang(122.0, 75, _TurnDir::CCW, 500, 1050);

					driveToPosition(0.5, -47.0, 25.0, -25.0, 0.5, 100, 1000, true, true, false);
				//	driveToDistance(22.5, pos.get_alpha(), 25.0, -24.0, 0.5, 100, 1050, true, true, false);
					//straight_line_drive(22.0,, float kP_correction, float maxErrA, int max_velocity, uint32_t settle, uint32_t max_time)
				//	float d, float a, float ys, float xs, float maxErrX, float maxVel, std::uint32_t ifailsafe, bool enableCorrect, bool forward, bool harshStop
					intake::light_sen(285);
					tilter::setTarget(tilter::State_Machine::E_STACK);
					pros::delay(2600);
>>>>>>> 92fd986464b0fdbbc43183bc402b23742a241a64
					intake::set_targetAsync(intake::States::E_OUTTAKE, 500);
					pros::delay(10);
					drive_lineup(-127, 500);
					break;

					case(2): // Red
					tilter::setTarget(tilter::State_Machine::E_LIFT);
				//	pros::delay(10);
<<<<<<< HEAD
				lift::setTargetAutonAsync(lift::heightsAUTO::E_MED, 1200);
			//	pros::delay(200);
				//intake::set_targetAsync(intake::States::E_OUTTAKE, 300);
				pros::delay(1200);
				tilter::setTarget(tilter::State_Machine::E_OFF);


					pros::delay(1600);
					intake::set_targetAsync(intake::States::E_INTAKE, 10000);
					driveToPosition(29.25, 0.0, 0.0, 0.0, 0.25, 90, 1800, true, true, false);
					driveToPosition(7.50, 23.0, 29.25, 0.0, 0.5, 90, 2000, true, false, false);
					turn2ang(0.5, 90, _TurnDir::CW, 500, 300);
					driveToPosition(48.0, 24.0, 7.50, 23.0, 0.5, 75, 2050, true, true, false);
					driveToPosition(25.0, 24.0, 48.0, 24.0, 0.5, 90, 1100, true, false, false);
					turn2ang(120.0, 75, _TurnDir::CW, 500, 1050);

					driveToPosition(0.5, 49.0, 25.0, 24.0, 0.5, 100, 1000, true, true, false);
				//	driveToDistance(22.5, pos.get_alpha(), 25.0, -24.0, 0.5, 100, 1050, true, true, false);
					//straight_line_drive(22.0,, float kP_correction, float maxErrA, int max_velocity, uint32_t settle, uint32_t max_time)
				//	float d, float a, float ys, float xs, float maxErrX, float maxVel, std::uint32_t ifailsafe, bool enableCorrect, bool forward, bool harshStop
				//	intake::light_sen(285);
					intake::set_targetAsync(intake::States::E_OUTTAKE, 350);
					tilter::setTarget(tilter::State_Machine::E_STACK);
					pros::delay(2800);
					intake::set_targetAsync(intake::States::E_OUTTAKE, 500);
					pros::delay(10);
					drive_lineup(-127, 500);
=======
					lift::setTargetAutonAsync(lift::heightsAUTO::E_LOW, 750);
					pros::delay(200);
					intake::set_targetAsync(intake::States::E_OUTTAKE, 300);
					pros::delay(675);
					tilter::setTarget(tilter::State_Machine::E_OFF);
					pros::delay(1700);
					intake::set_targetAsync(intake::States::E_INTAKE, 10000);
					driveToPosition(29.25, 0.0, 0.0, 0.0, 0.25, 90, 1800, true, true, false);
					driveToPosition(7.50, 25.0, 29.25, 0.0, 0.5, 90, 2000, true, false, false);
					turn2ang(0.0, 90, _TurnDir::CCW, 500, 300);
					driveToPosition(50.0, 25.0, 7.50, 25.0, 0.5, 75, 2050, true, true, false);
					driveToPosition(25.0, 25.0, 50.0, 25.0, 0.5, 90, 1100, true, false, false);
					turn2ang(122.0, 75, _TurnDir::CW, 500, 1050);

					driveToPosition(0.5, 47.0, 25.0, 25.0, 0.5, 100, 1000, true, true, false);
				//	driveToDistance(22.5, pos.get_alpha(), 25.0, -24.0, 0.5, 100, 1050, true, true, false);
					//straight_line_drive(22.0,, float kP_correction, float maxErrA, int max_velocity, uint32_t settle, uint32_t max_time)
				//	float d, float a, float ys, float xs, float maxErrX, float maxVel, std::uint32_t ifailsafe, bool enableCorrect, bool forward, bool harshStop
					intake::light_sen(285);
					tilter::setTarget(tilter::State_Machine::E_STACK);
					pros::delay(2600);
					intake::set_targetAsync(intake::States::E_OUTTAKE, 500);
					pros::delay(10);
					drive_lineup(-127, 500);

>>>>>>> 92fd986464b0fdbbc43183bc402b23742a241a64
					break;

					case(3): // 1 point
					drive_relative(25.0, 50.0);
					pros::delay(5000);
					drive_relative(-30.0, 50.0);
					pros::delay(3750);
					break;

					default:
<<<<<<< HEAD
					intake::light_sen(1000);
				/*	tilter::setTarget(tilter::State_Machine::E_LIFT);
				//	pros::delay(10);
					lift::setTargetAutonAsync(lift::heightsAUTO::E_MED, 1200);
				//	pros::delay(200);
					//intake::set_targetAsync(intake::States::E_OUTTAKE, 300);
					pros::delay(1200);
					tilter::setTarget(tilter::State_Machine::E_OFF);
					pros::delay(1700);
					intake::set_targetAsync(intake::States::E_INTAKE, 10000);
					driveToPosition(32.00, 0.0, 0.0, 0.0, 0.25, 90, 1850, true, true, false);
					driveToPosition(7.50, -28.5, 31.00, 0.0, 0.5, 90, 2000, true, false, false);
					turn2ang(0.0, 90, _TurnDir::CW, 500, 300);
					driveToPosition(50.0, -27.0, 7.50, -27.0, 0.5, 75, 2050, true, true, false);
					driveToPosition(25.0, -27.0, 50.0, -27.0, 0.5, 90, 1100, true, false, false);
					turn2ang(122.0, 75, _TurnDir::CCW, 500, 1000);

					driveToPosition(1.0, -50.0, 25.0, -28.5, 0.5, 110, 850, true, true, false);
				//	driveToDistance(22.5, pos.get_alpha(), 25.0, -24.0, 0.5, 100, 1050, true, true, false);
					//straight_line_drive(22.0,, float kP_correction, float maxErrA, int max_velocity, uint32_t settle, uint32_t max_time)
				//	float d, float a, float ys, float xs, float maxErrX, float maxVel, std::uint32_t ifailsafe, bool enableCorrect, bool forward, bool harshStop
					intake::light_sen(285);
					tilter::setTarget(tilter::State_Machine::E_STACK);
					pros::delay(2600);
					intake::set_targetAsync(intake::States::E_OUTTAKE, 500);
					pros::delay(10);
					drive_lineup(-127, 500);
					*/
=======
>>>>>>> 92fd986464b0fdbbc43183bc402b23742a241a64
					break;

}


//intake::set_targetAsync(intake::States::E_OUTTAKE, 1000);
//pros::delay(1000);
//	intake::light_sen(250);
/*
    tilter::setTarget(tilter::State_Machine::E_LIFT);
	//	pros::delay(10);
		lift::setTargetAutonAsync(lift::heightsAUTO::E_LOW, 750);
		pros::delay(200);
		intake::set_targetAsync(intake::States::E_OUTTAKE, 300);
		pros::delay(675);
		tilter::setTarget(tilter::State_Machine::E_OFF);
		pros::delay(1700);
		intake::set_targetAsync(intake::States::E_INTAKE, 10000);
    driveToPosition(29.25, 0.0, 0.0, 0.0, 0.25, 90, 1800, true, true, false);
    driveToPosition(7.50, -25.0, 29.25, 0.0, 0.5, 90, 2000, true, false, false);
		turn2ang(0.0, 90, _TurnDir::CW, 500, 300);
		driveToPosition(50.0, -25.0, 7.50, -25.0, 0.5, 75, 2050, true, true, false);
		driveToPosition(25.0, -25.0, 50.0, -25.0, 0.5, 90, 1100, true, false, false);
		turn2ang(123.5, 75, _TurnDir::CCW, 500, 1050);

		driveToPosition(0.5, -47.0, 25.0, -25.0, 0.5, 100, 1000, true, true, false);
	//	driveToDistance(22.5, pos.get_alpha(), 25.0, -24.0, 0.5, 100, 1050, true, true, false);
		//straight_line_drive(22.0,, float kP_correction, float maxErrA, int max_velocity, uint32_t settle, uint32_t max_time)
	//	float d, float a, float ys, float xs, float maxErrX, float maxVel, std::uint32_t ifailsafe, bool enableCorrect, bool forward, bool harshStop
		intake::light_sen(285);
		tilter::setTarget(tilter::State_Machine::E_STACK);
		pros::delay(2600);
		intake::set_targetAsync(intake::States::E_OUTTAKE, 500);
		pros::delay(10);
		drive_lineup(-127, 500);
		*/




	//	turn2ang()
	//	drive_lineup(-50, 1000);
		//turn2ang(0.0, 80, _TurnDir::CW, 300, 1000);








	//	tilter::setTarget(tilter::State_Machine::E_STACK);
//	readyToStack = true;
   //driveToPosition(8.0, 0.0, pos.get_y(), pos.get_x(), 0.1, 120.0, false, true, false);
//intake::deploy(1500);
	// if(light_sensor.get_value() > light_sensor_threshold){
		// intake_set(50);
	// }
/*
	auto chassis =
	ChassisControllerBuilder()
   .withMotors({8, 5}, {7, 6})
 //     {5, 8}, // Left motors are 1 & 2 (reversed)
     //  {6, 7})    // Right motors are 3 & 4
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 12.58_in}, imev5GreenTPR})
	.withMaxVelocity(150)
	.build();

	auto profileController =
	AsyncMotionProfileControllerBuilder()
    .withLimits({1.0, 2.0, 10.0})
    .withOutput(chassis)
    .buildMotionProfileController();

	profileController->generatePath({
  	{0_in, 0_in, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
  	{60_in, -43_in, 0_deg}}, // The next point in the profile, 3 feet forward
  	"A" // Profile name
	);

	//profileController->setTarget("A", true);
	//profileController->waitUntilSettled();

//	drive_set(50);
//turn2ang(130, 85, _TurnDir::CCW, 500, 1500);
//	intake::deploy(750);
	intake::set_targetAsync(intake::States::E_OUTTAKE, 1000);
	pros::delay(1000);
	intake::set_targetAsync(intake::States::E_INTAKE, 10000);
	straight_line_drive(52.0, 0.0, 175.0, 0.5, 85, 200, 2300);
	profileController->setTarget("A", true);
	profileController->waitUntilSettled();
	drive_lineup(-65, 500);
	pros::delay(50);


	straight_line_drive(48.0, 0.0, 200.0, 0.25, 80, 500, 2200);
	straight_line_drive(-20.0, 0.0, 100.0, 0.25, 80, 50, 500);
	pros::delay(50);
	turn2ang(137, 85, _TurnDir::CCW, 500, 1500);
	intake::set_targetAsync(intake::States::E_OUTTAKE, 200);
	straight_line_drive(58.0, -138.5, 150.0, 0.5, 127, 250, 1750);
	tilter::setTarget(tilter::State_Machine::E_STACK);
	pros::delay(2000);
//	outtake(-25, 150);
	straight_line_drive(-24, 0, 0, 0, 127, 0, 1000);



//	turn2ang(90.0, 100, _TurnDir::CW, 500, 2500);
	std::cout << "after AUTO " << radians_to_degrees(pos.get_alpha()) << std::endl << std::endl;
	//straight_line_drive(24,0,120,100,5000);

	*/
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

	opc_analog.set_t(12.5);
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


else if(light_sensor.get_value() > light_sensor_threshold2 && lift::g_clearTray &&  pros::millis() < lift::g_timeout){
lift::g_readyToLift = false;// && g_clearTray){ //&& pros::millis() < g_timeout){
int ispeed = pow(light_sensor.get_value()*0.01, 1.25);
ispeed = std::clamp(ispeed, -80, -60);

intake_set(ispeed);
}



else{
lift::g_clearTray = false;
lift::g_readyToLift = true;
intake_set(0);
}

<<<<<<< HEAD

=======
if(master_controller.get_digital(DIGITAL_Y)){
drive_set(-35);
intake_set(-50);
}
>>>>>>> 92fd986464b0fdbbc43183bc402b23742a241a64

std::cout << tilter_mtr.get_position() << std::endl << std::endl;


//if(master_controller.get_digital_new_press(DIGITAL_DOWN)){
//bool prezzed = !prezzed;
//if(prezzed)
//autonomous();
//}
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
	 std::cout << "LIGHT  " << light_sensor.get_value() << std::endl << std::endl;
<<<<<<< HEAD
	 //std::cout << "L  " << encoder360L.get_value() << std::endl << std::endl;
	 std::cout << "torq  " << tilter_mtr.get_torque() << std::endl << std::endl;
=======
	 std::cout << "L  " << encoder360L.get_value() << std::endl << std::endl;
	 std::cout << "R  " << encoder360R.get_value() << std::endl << std::endl;
>>>>>>> 92fd986464b0fdbbc43183bc402b23742a241a64
		pros::delay(10);


	}
}
