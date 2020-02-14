#include "motors.hpp"
#include "main.h"
// Motor constructor(uint8_t port, enum E_MOTOR_GEARSET, bool reverse, enum E_MOTOR_ENCODER)

pros::Motor LF_mtr (8, pros::E_MOTOR_GEARSET_18, false , pros::E_MOTOR_ENCODER_COUNTS); //false  10 10
pros::Motor RF_mtr (7, pros::E_MOTOR_GEARSET_18, true , pros::E_MOTOR_ENCODER_COUNTS); // port 12 true
pros::Motor RIntake_mtr (3, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor LIntake_mtr (10, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor tilter_mtr (12, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor RB_mtr (6, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS); // [port 4] true
pros::Motor LB_mtr (5, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);// port 5 false
pros::Motor lift_mtr (4, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_COUNTS);

float encoder_avg_mtrs(){
return (LF_mtr.get_position() + RF_mtr.get_position() + RB_mtr.get_position() + LB_mtr.get_position()) / 4;
}

float encoder_lift(){
return lift_mtr.get_position();
}

float encoder_intake_left(){
return LIntake_mtr.get_position();
}

float encoder_intake_right(){
return RIntake_mtr.get_position();
}

void reset_mtr_encoders()
{
LF_mtr.tare_position();
RF_mtr.tare_position();
LB_mtr.tare_position();
RB_mtr.tare_position();
}

void driveLR_vel_set(int velL, int velR){
  LF_mtr.move_velocity(velR);
  RF_mtr.move_velocity(velR);
  LB_mtr.move_velocity(velL);
  RB_mtr.move_velocity(velR);
}

void driveLR_set(int voltageL, int voltageR){
  LF_mtr.move(voltageL);
  RF_mtr.move(voltageR);
  LB_mtr.move(voltageL);
  RB_mtr.move(voltageR);
}

void drive_set(int voltage){
  LF_mtr.move(voltage);
  RF_mtr.move(voltage);
  LB_mtr.move(voltage);
  RB_mtr.move(voltage);
}

void drive_setV(int velocity){
  LF_mtr.move_velocity(velocity);
  RF_mtr.move_velocity(velocity);
  LB_mtr.move_velocity(velocity);
  RB_mtr.move_velocity(velocity);
}

void intake_set(int voltage){
RIntake_mtr.move(voltage);
LIntake_mtr.move(voltage);
}

void lift_set(int voltage){
lift_mtr.move(voltage);
}

void tilter_set(int voltage){
tilter_mtr.move(voltage);
}


void left_intake_set(int voltage){
LIntake_mtr.move(voltage);
}

void right_intake_set(int voltage){
RIntake_mtr.move(voltage);
}


int drive_setR(int voltage){
  LF_mtr.move(voltage);
  RF_mtr.move(voltage);
  LB_mtr.move(voltage);
  RB_mtr.move(voltage);
	return voltage;
}

void leftdrive_set(int voltage){
  LF_mtr.move(voltage);
  LB_mtr.move(voltage);
}

void rightdrive_set(int voltage){
  RF_mtr.move(voltage);
  RB_mtr.move(voltage);
}


int leftdrive_setR(int voltage){
  LF_mtr.move(voltage);
  LB_mtr.move(voltage);
	return voltage;
}

int rightdrive_setR(int voltage){
  RF_mtr.move(voltage);
  RB_mtr.move(voltage);
	return voltage;
}

void turning_set(int voltage){
  LF_mtr.move(voltage);
  RF_mtr.move(-voltage);
  LB_mtr.move(voltage);
  RB_mtr.move(-voltage);
}
