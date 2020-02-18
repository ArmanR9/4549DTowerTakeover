#ifndef MOTORS_HPP
#define MOTORS_HPP
#include "main.h"

// *-------------------------------------- *
// *                                       *
// * Declaring motors/sensors & paramaters *
// *                                       *
// *---------------------------------------*

extern pros::Motor LF_mtr;
extern pros::Motor RF_mtr;
extern pros::Motor LIntake_mtr;
extern pros::Motor RIntake_mtr;
extern pros::Motor RB_mtr;
extern pros::Motor LB_mtr;
extern pros::Motor lift_mtr;
extern pros::Motor tilter_mtr;

// *----------------------------------- *
// *                                    *
// *      Functions for motors          *
// *                                    *
// *------------------------------------*

float encoder_avg_mtrs();
float encoder_lift();
float encoder_LIntake();
float encoder_RIntake();


void reset_mtr_encoders();

void brake_mode_init();

void driveLR_vel_set(int velL, int velR);
void driveLR_set(int voltageL, int voltageR);
void turning_set(int voltage);
void drive_set(int voltage);
void drive_setV(int velocity);

void lift_set(int voltage);
void intake_set(int voltage);
void tilter_set(int voltage);

void left_intake_set(int voltage);
void right_intake_set(int voltage);

void leftdrive_set(int voltage);
void rightdrive_set(int voltage);

int leftdrive_setR(int voltage);
int rightdrive_setR(int voltage);
int drive_setR(int voltage);


#endif
