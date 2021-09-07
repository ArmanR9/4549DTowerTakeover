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


void deployAuto(){

	tilter::setTarget(tilter::State_Machine::E_LIFT);
	lift::setTargetAutonAsync(lift::heightsAUTO::E_MED, 1200);

	pros::delay(1200);

	tilter::setTarget(tilter::State_Machine::E_OFF);

	pros::delay(1600);
}

void tower(){
    intake::light_senAsync(250, -115, -105);
	pros::delay(750);
	intake::light_senOff();
	lift::autonLift(lift::heightsAUTO::E_LOW, 2500, 2000);
	pros::delay(1000);
	intake::set_targetAsync(intake::States::E_DEPLOY, 1000);
	pros::delay(1950);
	tilter::setTarget(tilter::State_Machine::E_OFF);
}

void tower2(){
    intake::light_senAsync(300, -115, -105);
	pros::delay(750);
	intake::light_senOff();
	lift::autonLift(lift::heightsAUTO::E_MED, 2500, 2000);
	pros::delay(1200);
	intake::set_targetAsync(intake::States::E_DEPLOY, 1000);
	pros::delay(1950);
	tilter::setTarget(tilter::State_Machine::E_OFF);
}

void stack(){

	intake::set_targetAsync(intake::States::E_OFF, 10);
	pros::delay(20);
	intake::light_senAsync(300, -605, -60);
				
	pros::delay(300);
	intake::light_senOff();
		
	tilter::setTarget(tilter::State_Machine::E_STACK, 15000);
				
	pros::delay(3000);
					
}

void stack2(){

	intake::set_targetAsync(intake::States::E_OFF, 10);
	pros::delay(20);
	intake::light_senAsync(150, -40, -35);
				
	pros::delay(150);
	intake::light_senOff();
	intake::set_targetAsync(intake::States::E_INTAKE, 50);
	
	tilter::setTarget(tilter::State_Machine::E_STACK, 15000);
				
	pros::delay(3000);

	drive_lineup(35, 200);
}


void autonomous(){

	// Reset odometry data again as an insurance.
	pos.reset_pos();
	pros::delay(10);

 	switch(auton_sel){

					case(1): // Blue

						deployAuto();
						intake::set_targetAsync(intake::States::E_INTAKE, 10000);
						driveToPosition(32.0, 0.0, 0.0, 0.0, 0.05, 80, 1800, true, true, false);
						driveToPosition(9.0, -27.0, 32.0, 0.0, 0.05, 100, 2000, true, false, false);
						turn2ang(0.0, 120, _TurnDir::CH, 500, 1000);
						driveToPosition(41.8, -27.7, 9.0, -27.0, 0.05, 80, 2000, true, true, false);
						driveToPosition(30.2, -27.7, 41.8, -27.7, 0.05, 90, 2000, true, true, false);
						turn2ang(120, 120, _TurnDir::CCW, 500, 1000);
						driveToPosition(0.5, -48.5, 30.2, -27.7, 0.05, 127, 1000, true, true, false);
						stack();
						pros::delay(2700);

						drive_lineup(-90, 300);

						deployAuto();

						intake::set_targetAsync(intake::States::E_INTAKE, 10000);
						driveToPosition(29.25, 0.0, 0.0, 0.0, 0.25, 90, 1800, true, true, false);
						driveToPosition(7.50, -22.0, 29.25, 0.0, 0.5, 90, 2000, true, false, false);
						driveToPosition(48.5, -24.0, 7.50, -23.0, 0.5, 75, 2150, true, true, false);
						driveToPosition(25.0, -24.0, 48.0, -24.0, 0.5, 90, 1100, true, false, false);
						turn2ang(120.0, 75, _TurnDir::CCW, 500, 1050);

						driveToPosition(0.5, -48.5, 25.0, -24.0, 0.5, 100, 1000, true, true, false);
						stack();
						pros::delay(2800);

						drive_lineup(-90, 500);
						break;

					case(2): // Red

						tilter::setTarget(tilter::State_Machine::E_LIFT);
						lift::setTargetAutonAsync(lift::heightsAUTO::E_MED, 1200);
						pros::delay(1200);

						tilter::setTarget(tilter::State_Machine::E_OFF);
						pros::delay(1600);

						intake::set_targetAsync(intake::States::E_INTAKE, 10000);
						driveToPosition(29.25, 0.0, 0.0, 0.0, 0.25, 90, 1800, true, true, false);
						driveToPosition(7.50, 22.0, 29.25, 0.0, 0.5, 90, 2000, true, false, false);
						driveToPosition(48.0, 24.0, 7.50, 23.0, 0.5, 75, 2550, true, true, false);
						driveToPosition(15.0, 24.0, 48.0, 24.0, 0.5, 90, 2000, true, false, false);
						turn2ang(120.0, 75, _TurnDir::CW, 500, 1050);

						driveToPosition(0.5, 45.0, 25.0, 24.0, 0.5, 100, 1000, true, true, false);
		
						stack();
			
						pros::delay(2800);
						intake::set_targetAsync(intake::States::E_OUTTAKE, 500);
						pros::delay(10);

						drive_lineup(-30, 1000);
						pros::delay(1000);

						drive_lineup(-90, 500);
						break;


					case(3): // 1 point auto

						drive_relative(25.0, 50.0);
						pros::delay(5000);
						drive_relative(-30.0, 50.0);
						pros::delay(3750);
						break;

					case(4): // Backup

						intake::set_targetAsync(intake::States::E_INTAKE, 12000);
						driveToPosition(27.0, 0.0, 0.0, 0.0, 0.05, 90, 1000, true, true, false);
						driveToPosition(17.0, 0.0, 27.0, 0.0, 0.05, 110, 900, true, false, false);
						turn2ang(91.0, 100, _TurnDir::CCW, 500, 1000, 2.0);
						driveToPosition(21.0, -36.0, 17.0, 27.0, 0.05, 110, 900, true, true, false);
						driveToPosition(21.0, -19.0, 17.0, 27.0, 0.05, 110, 900, true, false, false);
						turn2ang(91.0, 100, _TurnDir::CW, 500, 1000, 2.0);
						driveToPosition(9.25, 5.33, 21.0, -19.0, 0.05, 110, 900, true, true, false);
						intake::set_targetAsync(intake::States::E_OFF, 20);
						stack();
						drive_lineup(-50, 300);
						break;

					default: // Skills

						turn2ang(0.0, 127, _TurnDir::CH, 300, 1550);
						turning_lineup(-50, 200);
						pros::delay(1000);

						lift::autonLift(lift::heightsAUTO::E_MED, 5000, 5850);
						pros::delay(1000);

						drive_lineup(60, 1000);
						pros::delay(1000);

						pos.reset_pos();
						pros::delay(1000);
					
						driveToPosition(-12.0, 0.0, 0.0, 0.0, 0.05, 100, 3000, true, false, false);
						turn2ang(90.0, 110, _TurnDir::CCW, 600, 1300);
						intake::set_targetAsync(intake::States::E_INTAKE, 4000);
						driveToPosition(-12.0, -32.0, -12.0, 0.0, 0.05, 100, 3000, true, true, false);
						driveToPosition(-15.0, -27.5, -12.0, -32.0, 0.05, 100, 3000, true, false, false);
						tower2();
						pros::delay(100000);

						deployAuto();

						// Tower and Reset to the wall
						intake::set_targetAsync(intake::States::E_INTAKE, 2000);
						driveToPosition(10.0, 0.0, 0.0, 0.0, 0.05, 80, 1000, true, true, false);
						turn2ang(45.0, 90, _TurnDir::CW, 500, 1500);
					
						driveToPosition(19.25, 12.00, 10.0, 0.0, 0.05, 70, 1750, true, true, false);
						tower();
						driveToPosition(0.0, 0.0, 19.25, 12.00, 0.05, 90, 1500, true, false, false);
						turn2ang(0.0, 100, _TurnDir::CW, 550, 1000 , 5.25);
						pos.reset_pos();
						pros::delay(100);

						// Go for the 8 cube lineup
						intake::set_targetAsync(intake::States::E_INTAKE, 16500);
						driveToPosition(63.5, 0.5, 0.0, 0.0, 0.05, 70, 4000, true, true, false);
						turn2ang(0.0, 70, _TurnDir::CH, 500, 2000);
						tilter::setTarget(tilter::State_Machine::E_LIFT, 20000);
						driveToPosition(135.0, 1.0, 63.5, 0.5, 0.05, 70, 4000, true, true, false);

						// Align for scoring zone
						turn2ang(60.0, 90, _TurnDir::CW, 500, 1300);
						driveToPosition(145.0, 15.00, 135.0, 1.0, 0.05, 127, 3000, true, true, false);
						tilter::setTarget(tilter::State_Machine::E_OFF);
						intake::set_targetAsync(intake::States::E_INTAKE, 1500);
						drive_lineup(50, 100);
						pros::delay(2300);

						stack2();
						pros::delay(3000);

						drive_lineup(-85, 125);
						pros::delay(1000);

						tilter::setTarget(tilter::State_Machine::E_OFF);
						turn2ang(0.0, 127, _TurnDir::CH, 300, 1550);
						turning_lineup(-50, 200);
						pros::delay(1000);

						lift::autonLift(lift::heightsAUTO::E_MED, 5000, 5850);
						pros::delay(1000);

 	        		    drive_lineup(60, 1550);
						pros::delay(1000);

						pos.reset_pos();	
						pros::delay(1000);

						driveToPosition(-12.0, 0.00, 0.0, 0.0, 0.05, 90, 1250, true, false, false);
						pros::delay(2000);
	

				
						turn2ang(90.0, 110, _TurnDir::CCW, 600, 1300);
						intake::set_targetAsync(intake::States::E_INTAKE, 4000);
						driveToPosition(-13.0, -32.0, -13.0, 0.0, 0.05, 100, 3000, true, true, false);
						driveToPosition(-13.0, -27.5, -13.0, -32.0, 0.05, 100, 3000, true, false, false);
						tower2();
						turn2ang(0.0, 90.0, _TurnDir::CH, 500, 3000, 3.1);
						drive_lineup(-90, 1000);
						pos.reset_pos();
						pros::delay(500);

						// Get the other stack
						driveToPosition(63.5, -0.5, 0.0 ,0.0, 0.05, 100, 2750, true, true, false);
						turn2ang(0.0, 70, _TurnDir::CH, 500, 1250);
						tilter::setTarget(tilter::State_Machine::E_LIFT, 20000);
						driveToPosition(135.5, -0.5, 63.5, -0.5, 0.05, 70, 4250, true, true, false);
						turn2ang(55.0, 90, _TurnDir::CCW, 500, 2000);
						driveToPosition(142.0, -35.0, 135.5, -0.5, 0.05, 90, 6000, true, true, false);
						stack();
						pros::delay(2000);
	
						drive_lineup(-50, 500);
						turn2ang(0.0,110, _TurnDir::CH, 500, 2000);
						lift::autonLift(lift::heightsAUTO::E_MED, 5000, 5850);
						pros::delay(1000);
		
					    drive_lineup(90, 1000);
						pros::delay(1000);

						pos.reset_pos();
						pros::delay(100);

						driveToPosition(-15.0, 0.0, 0.0, 0.0, 0.05, 100, 3000, true, false, false);
						turn2ang(90.0, 110, _TurnDir::CW, 550, 1200, 2.5);
						intake::set_targetAsync(intake::States::E_INTAKE, 4000);
						driveToPosition(-15.0, -20.0, -15.0, 0.0, 0.05, 100, 3000, true, true, false);
						driveToPosition(-15.0, -15.0, -15.0, -20.0, 0.05, 100, 3000, true, false, false);
						tower2();				
						break;
				}

}