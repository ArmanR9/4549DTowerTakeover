#include "sensors.hpp"
#include "main.h"
#include "motors.hpp"
#include "okapi/api.hpp"

pros::ADIEncoder encoder360L(1,2, false);
pros::ADIEncoder encoder360R(5,6, true);
pros::ADIEncoder encoder360B(3,4, false);
pros::ADIAnalogIn light_sensor(7);

pros::Controller master_controller(pros::E_CONTROLLER_MASTER);

float encoder360LTicks = encoder360L.get_value(); 
float encoder360RTicks = encoder360R.get_value();

float encoder360avg_in(){
    return (encoder360LIn + encoder360RIn) / 2.0;
}

void reset_encoders()
{
    encoder360L.reset();
    encoder360R.reset();
    encoder360B.reset();
}

void reset_sensors(){
    reset_encoders();
    reset_mtr_encoders();
}

// Redundant abstraction, but it makes calling the verbose methods much easier to read

int getJoyLY(pros::Controller controller_type){
    return controller_type.get_analog(ANALOG_LEFT_Y);
}

int getJoyRY(pros::Controller controller_type){
    return controller_type.get_analog(ANALOG_RIGHT_Y);
}

int getJoyLX(pros::Controller controller_type){
    return controller_type.get_analog(ANALOG_LEFT_X);
}

int getJoyRX(pros::Controller controller_type){
    return controller_type.get_analog(ANALOG_RIGHT_X);
}
