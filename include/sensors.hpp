#ifndef SENSORS_HPP
#define SENSORS_HPP
#include "main.h"



// *---------------------------------------------*s
// *                                             *
// * Declaring sensors/mischalenous & paramaters *
// *                                             *
// *---------------------------------------------*

extern pros::ADIPort lift_pot;
extern pros::ADIPort tilter_pot;
extern pros::ADIEncoder encoder360R;
extern pros::ADIEncoder encoder360L;
extern pros::ADIEncoder encoder360B;
extern pros::Controller master_controller;

// *--------------------------------------*
// *                                      *
// *    Functions/imprtant variables      *
// *     for algorithims (PID,Odom)       *
// *                                      *
// *--------------------------------------*

extern float encoder360LTicks;
extern float encoder360RTicks;

extern float encoder360LIn;
extern float encoder360RIn;

// *--------------------------------------*
// *                                      *
// *     Sensor/Controller Functions      *
// *                                      *
// *--------------------------------------*
float encoder360avg_in();

void reset_encoders();
void reset_sensors();

int getJoyLY(pros::Controller controller_type);
int getJoyRY(pros::Controller controller_type);
int getJoyLX(pros::Controller controller_type);
int getJoyRX(pros::Controller controller_type);
#endif
