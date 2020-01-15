#include "auto_drive.hpp"
#include "main.h"
#include "motors.hpp"
#include "sensors.hpp"
#include "utilities.hpp"
#include "odometry.hpp"

static float g_currError{0.0};
static bool isAccel_done{false};
static bool isPID_done{false};

void drive_coordinator(float setpoint, float velocity, float voltage){
    motion_profile(setpoint, velocity);
    //blocking???
    drive_encoder(setpoint, voltage, g_currError);

//block from anyother functions being called
  while(!isAccel_done || !isPID_done){
    pros::delay(10);
  }
}

void motion_profile(double setpoint){
Velocity profile;
isAccel_done = false; // Set flag to false as we are beginning the acceleration stage.

// Bot's constraints; max jerk, accel, and velocity
const double maxJerk = 0.1;
const double maxAccel = 5.0;
const double maxVel = 200;

if(fabs(setpoint) > 200) setpoint = maxVel * sgn_(setpoint); // If over max velocity, adjust to +-200RPM

double accel = 0.0; // intialize vel,accel,jerk is 0
double vel = 0.0;
double jerk = 0.0;

while (vel < setpoint){ // Until current velocity is at cruising speed, accelerate

  vel = profile.get_vel();
  accel = profile.get_accel(); // Set vel, accel, jerk all to current bot's.
  jerk = profile.get_jerk();

    if (accel < maxAccel) accel = std::min(accel + jerk, maxAccel); // Keep increasing the accleration by the jerk (limit accel not to go over constraint)
    vel = std::min(vel + accel, setpoint); // Add the acceleration to the velocity until it peaks to max allowable velocity
    drive_setV(vel); // Drive at the current computed velocity
    pros::delay(10);
}
//g_currError = error;
isAccel_done = true; // Acceleration is done, enable flag and start normal motion algorithim.


// send target as global, calculate the error during this function, do target -= target - error to find remaining target left, and feed it into drive_encoder
}

void drive_point(double targetX, double targetY, double startX = pos.get_x(), double startY = pos.get_y(), double startA = pos.get_alpha(), float velocity = 120, uint32_t settle = 1000, uint32_t max_time = 5000){
Vector _currVector;
Polar  _currPolar;

double errorX, errorY, errorA;

pos.set_x(startX);
pos.set_y(startY);
pos.set_alpha(startA);

double currX = startX;
double currY = startY;

double currVector_x = startX;
double currVector_y = startY;

double pX, iX, dX;
double pY, iY, dY;
double pA, iA, dA;

bool i_zone = false;
float i_limit = 50.0;
double last_d;

float kP = 0.5;
float kD = 0.0;
float kI = 0.0;

uint32_t timer{0};
uint32_t _last_time{0};
bool invoke_timer;
uint32_t failsafe = pros::millis() + max_time;

float threshold = 1.0; // 1 inch

float final_power;
float max_power_abs;
float max_vel_abs = fabs(velocity);

double targetA = lineAngle(targetX, targetY, startX, startY);

while(pros::millis() < timer && pros::competition::is_autonomous() && pros::millis() < failsafe){
currVector_x += pos.get_x() - startX;
currVector_y += pos.get_y() - startY;

 _currVector.x = pos.get_x() - targetX;
 _currVector.y = pos.get_y() - targetY;
 _currPolar = vector_to_polar(_currVector, _currPolar);
 _currPolar.theta += targetA;
 _currVector = polar_to_vector(_currPolar, _currVector);



//currX += pos.x;
//currY += pos.y;

errorA = targetA - pos.get_alpha();
errorX = targetX - currVector_x;
errorY = targetY - currVector_y;

//pX = errorX * kP;
pY = errorY * kP;
//pA = 0.0;//errorA * kP;

final_power = pX + pY + pA;
max_power_abs = fabs(final_power);

if(max_power_abs > max_vel_abs){
final_power = max_vel_abs * sgn_(velocity);
}


if(i_zone){

}

else { iX = 0.0; }

    if(fabs(errorY) || fabs(errorX) < threshold){
     invoke_timer = true;
     i_zone = false;
    }

    else{ invoke_timer = false;}

    if (invoke_timer){ timer = pros::millis() + settle;}

    drive_set(final_power);

pros::delay(10);
}
drive_set(0);
}



void drive_encoder(int setpoint, int max_speed, int currError){
//unsigned long time{0};
//unsigned long last_time{0};
//unsigned long dT{0};
std::uint32_t _time{0};
std::uint32_t _last_time{0};
std::uint32_t _dT{0};

float error {0.0};
float error2 {0.0};
float encoder360_average;
float proportional, integral, derivative;
bool integral_active_zone = 0;
float integral_limit = 50.0;
float filtered_derivative{0.0};
float last_filtered_derivative{0.0};
float last_error{0.0};
float last_error2{0.0};
float accError{0.0};

float position{0.0};
float last_position{0.0};

float final_power{0.0};
float final_powerABS {0.0};
int max_speedABS = abs(max_speed);

float kP = 0.8;
float kD = 8.0; //use X.0 for derivitive as dT is being calculated in milliseconds and makes the value very small
float kI = 0.0;

while(true){
//time = pros::millis();
//dT = time - last_time;
_time = pros::millis();
_dT = _time - _last_time;



if(_dT < 10) { _dT = 10; }
else if(_dT > 30) { _dT = 30; }


float encoder_average = ((LF_mtr.get_position() + RF_mtr.get_position() + RB_mtr.get_position() + LB_mtr.get_position())/4.0)/900.0*(circ);
float encoder360_average = ((encoder360R.get_value() + encoder360L.get_value()) / 2.0)/360.0*(circ);
error = (setpoint - encoder360_average);
error2 = (setpoint - encoder_average);

position = encoder_average; // encoder360_average

//error = (setpoint/360*(CIRC) - encoder360_average);

if(fabs(error) > (6)) { integral_active_zone = false; }


else{ integral_active_zone = true; }




if(sgn_(final_power) == 1 && integral_active_zone ) {
accError += error;
}

else if(sgn_(final_power) == -1 && integral_active_zone ) //-error > IntegralActiveZone
{
accError -= error;
}


if(accError > 50) //IntegralLimit  50/ki
{
accError = integral_limit;
}

else if(accError < -50) //-IntegralLimit  -50/ki
{
accError = -integral_limit;
}                     //accError > IntegralLimit

printf("error val %f\n", error2);
printf("derivitive val %f\n", derivative);
printf("Integral val %f\n", integral);

derivative = (position - last_position)/_dT;

proportional = error2;
filtered_derivative = (derivative*0.6 + last_filtered_derivative*0.4);
integral = accError;   // * or /_dT;


final_power = kP*proportional + kD*filtered_derivative; //+ kI*Integral;


float final_powerABS = fabs(final_power);

if(final_powerABS > 127.0){
final_power = 127.0 * final_power / final_powerABS;
}

else if(final_powerABS > max_speedABS){
final_power = max_speedABS * final_power / final_powerABS;
}





printf("final_power val %f\n", final_power);


drive_set((int)final_power);

last_error = error;
last_error2 = error2;
last_position = position;
last_filtered_derivative = filtered_derivative;
//last_time = time;
_last_time = _time;

printf("dT %u\n", _dT);
printf("updated time %u\n", _time);
printf("pros millis %d\n", pros::millis());
printf("last time %u\n", _last_time);

// %lu

pros::delay(10);
//std::uint32_t time = pros::millis();

//pros::Task::delay_until(&_time, 10);

}
pros::lcd::print(6,"Distance travled %f\n", ((encoder360R.get_value() + encoder360L.get_value()) / 57.3248407644));
drive_set(0);
}



void arc_turns(int setpoint, int max_speed, unsigned int timeout, int direction){
float error;
float arc_outer;
float arc_inner;
bool exitLoop = false;


while(!exitLoop){

switch(direction){

case(LEFT_DIRECTION):
// RIGHT ARC(OUTER ARC) - LEFT SIDE (INNER ARC)
//we get arc angle and make swing turn

break;

case(RIGHT_DIRECTION):
break;

default:
exitLoop = true;
}


}
}

void drive_pid(int setpoint, int max_speed, unsigned int timeout){

  float error;
  float encoder_average;
  float encoder_flywheel;
  float proportional, integral, derivative;
  bool IntegralActiveZone;
  float IntegralLimit = 50.0;
  float encoder_avgL;
  float encoder_avgR;
  float encoder_arm;
  float final_power;
  float encoder_back;


  float last_error;
  float accError;
  bool forward_direction;
  float tick;
  //bool exitLoop;
  int failsafe = 2500;
  int intial_millis = pros::millis();
  unsigned int net_timer;
  bool timer_drive = true;
  float encoder360l;
  float encoder360r;

  net_timer = pros::millis() + timeout;

  if (max_speed > 110) {
    max_speed = 110;
  } else if (max_speed < -110) {
    max_speed = -110;
  }

/*  if (setpoint > 0) {
    forward_direction = true;
  } else if (setpoint < 0) {
    forward_direction = false;
  }
*/


   float kP = 0.2; //0.301
   float kD = 0.0;
   float kI = 0.0;

  while(true) {  // ((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis()))

    // printf("test + %f\n", pos.a);

     encoder_back = (LF_mtr.get_position() + RF_mtr.get_position())/2.0;
     encoder_average = (LF_mtr.get_position() + RF_mtr.get_position() + RB_mtr.get_position() + LB_mtr.get_position())/4.0;
     encoder_avgL = (LF_mtr.get_position() + LB_mtr.get_position())/2.0;
     encoder_avgR = (RF_mtr.get_position() + RB_mtr.get_position()/2.0);
     error = ((setpoint/circ) * 900.0) - encoder_average; //900 = amount of rotations to drive one inch || (setpoint/4*pi) * 900)
     //(12/(4*pi))
     printf("IntegralActiveZone %c\n", IntegralActiveZone);

     if (fabs(error) > (12/circ)*900.0){
     IntegralActiveZone = false;
     }

     else{
     IntegralActiveZone = true;
     }



  if(forward_direction == true) {
     accError += error;
   }

   if(forward_direction == false) //-error > IntegralActiveZone
   {
   accError -= error;
   }


   if(accError > 50.0) //IntegralLimit  50/ki
   {
   accError = IntegralLimit;
   }

   if(accError < -50.0) //-IntegralLimit  -50/ki
   {
   accError = -IntegralLimit;
   }                     //accError > IntegralLimit

    printf("error val %f\n", error);
    printf("derivitive val %f\n", derivative);
    printf("Integral val %f\n", integral);

    proportional = kP*error;
    derivative = kD*(error - last_error);
    integral = kI*accError;

    final_power = proportional + derivative;

    if(final_power > max_speed)
    {
    final_power = max_speed;
    }

    if(final_power < -max_speed)
    {
    final_power = -max_speed;
    }

    printf("final_power val %f\n", final_power);


     last_error = error;


     drive_set(final_power);



     pros::delay(10);
  }


pros::lcd::print(7,"Distance travled %f\n", ((LF_mtr.get_position() + RF_mtr.get_position() + RB_mtr.get_position() + LB_mtr.get_position())/286.47889)); //

drive_set(0);
}

//---- Shaft Encoders (4"*pi) = CIRC -----------
// 360 = 1 rotation where 12.56 inches travelled || 28.6624203822 ticks to travel one inch
//28.6*2 = 57.3248407644 to find how many inches travelled (divide)

//----- Shaft Encoders (3.25"*pi) = CIRC ---------
// 360 = 1 rotation where 10.2101761242 inches travelled || 35.2589412388 ticks to travel one inch
// 35.2*2 = 70.5178824776 to find how many inches travelled (divide)

//------ Shaft Encoders (2.75"*pi) = CIRC --------
// 360 = 1 rotation where 8.63937979737 inches travelled || 41.6696578277 ticks to travel one inch
// 41.6*2 = 83.3393156554 to find how many inches travelled (divide)

//---- Motor Encoders (4*pi) = CIRC -----------
//900 = 1 rotation which = 12.56 inches travelled || 71.6197244914 ticks to travel one inch
//void inches(int multiplier){
//71.6197244914*multiplier;
//71*4 = 286 so find how many inches travelled (divide)
//}




void turning_pid(float setpoint, int max_speed, unsigned int timeout){

  float error;
  float encoder_average;
  float encoder_flywheel;
  float proportional, integral, derivative;
  bool IntegralActiveZone;
  float IntegralLimit = 30.0;
  float encoder_avgL;
  float encoder_avgR;
  float encoder_arm;
  float final_power;
  float encoder_back;

  float last_error;
  float accError;
  bool left_direction;
  float tick;
  int failsafe = 2500;
  int intial_millis = pros::millis();
  unsigned int net_timer;
  bool timer_drive = true;
  float ticks_to_degrees = 0.0;

  net_timer = pros::millis() + timeout;

  if (max_speed > 110) {
    max_speed = 110;
  } else if (max_speed < -110) {
    max_speed = -110;
  }

  if (setpoint > 0.0) {
    left_direction = true;
  } else if (setpoint < 0.0) {
    left_direction = false;
  }

  while(true) {  // ((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis()))

    float kP = 0.18; //0.301
    float kD = 0.5;
    float kI = 0;

     encoder_back = (LB_mtr.get_position() + RB_mtr.get_position())/2.0;
     encoder_average = (LB_mtr.get_position() + RB_mtr.get_position() + LF_mtr.get_position() + RF_mtr.get_position())/4.0;
     encoder_avgL = (LB_mtr.get_position() + LF_mtr.get_position())/2.0;
     encoder_avgR = (RB_mtr.get_position() + RF_mtr.get_position()/2.0);

      //calling the orientation function to make it's calculations

     error = setpoint - pos.get_alpha();

     //degrees * ticks_to_degrees
     printf("IntegralActiveZone %c\n", IntegralActiveZone);

     if (fabs(error) > (30.0*ticks_to_degrees)){
     IntegralActiveZone = false;
     }

     else{
     IntegralActiveZone = true;
     }


  if(left_direction == true) {
     accError += error;
   }

   if(left_direction == false)
   {
   accError -= error;
   }


   if(accError > 30.0)
   {
   accError = IntegralLimit;
   }

   if(accError < -30.0)
   {
   accError = -IntegralLimit;
   }

    printf("error val %f\n", error);
    printf("derivitive val %f\n", derivative);
    printf("Integral val %f\n", integral);

    proportional = error*kP;
    derivative = (error - last_error)*kD;
    integral = accError*kI;

    final_power = proportional + derivative;

    if(final_power > max_speed)
    {
    final_power = max_speed;
    }

    if(final_power < -max_speed)
    {
    final_power = -max_speed;
    }

    printf("final_power val %f\n", final_power);

     if (fabs(error) < (2.0*ticks_to_degrees)){
     timer_drive = false;
     }
     else{
     net_timer = pros::millis() + timeout;
     }


     last_error = error;



     turning_set(final_power);



  pros::delay(10);
}

pros::lcd::print(7,"Distance travled %f\n", ((LB_mtr.get_position() + RB_mtr.get_position() + RB_mtr.get_position() + LB_mtr.get_position())/286.47889)); //286.5

turning_set(0);
}



void straight_line_drive(float setpoint, int max_speed, unsigned int timeout, float angle){


  float correction_error;
  float error;
  float encoder_average;
  float encoder_flywheel;
  float proportional, integral, derivative;
  float proportional_correction;
  bool IntegralActiveZone;
  float IntegralLimit = 50.0;
  float encoder_avgL;
  float encoder_avgR;
  float encoder_arm;
  float final_power;
  float encoder_back;

  float last_error;
  float accError;
  bool forward_direction;
  int failsafe = 2500;
  int intial_millis = pros::millis();
  unsigned int net_timer;
  bool timer_drive = true;
  float encoder360l;
  float encoder360r;

  net_timer = pros::millis() + timeout;

  if (max_speed > 110) {
    max_speed = 110;
  } else if (max_speed < -110) {
    max_speed = -110;
  }

  if (setpoint > 0.0) {
    forward_direction = true;
  } else if (setpoint < 0.0) {
    forward_direction = false;
  }


   float kP = 0.2; //0.301
   float kP_correction = 0.0;
   float kD = 0.8;
   float kI = 0.0;

  while(true) {  // ((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis()))
    float encoder360L_ticks = encoder360L.get_value();
    float encoder360R_ticks = encoder360R.get_value();

    float current_orientation_PID = ((encoder360L_ticks/360.0*(circ)) - (encoder360R_ticks/360.0*(circ)) / (2.0*chassis_width))*180.0/M_PI;

     encoder_back = (LF_mtr.get_position() + RF_mtr.get_position())/2.0;
     encoder_average = (LF_mtr.get_position() + RF_mtr.get_position() + RB_mtr.get_position() + LB_mtr.get_position())/4.0;
     encoder_avgL = (LF_mtr.get_position() + LB_mtr.get_position())/2.0;
     encoder_avgR = (RF_mtr.get_position() + RB_mtr.get_position()/2.0);

     correction_error = (angle + current_orientation_PID);
     printf("current orientation %f\n", current_orientation_PID);
     error = ((setpoint/circ) * 900.0) - encoder_average;

     printf("IntegralActiveZone %c\n", IntegralActiveZone);

     if (fabs(error) > (12.0/circ)*900.0){
     IntegralActiveZone = false;
     }

     else{
     IntegralActiveZone = true;
     }



  if(forward_direction == true) {
     accError += error;
   }

   if(forward_direction == false)
   {
   accError -= error;
   }


   if(accError > 50.0)
   {
   accError = IntegralLimit;
   }

   if(accError < -50.0)
   {
   accError = -IntegralLimit;
   }

    printf("error val %f\n", error);
    printf("derivitive val %f\n", derivative);
    printf("Integral val %f\n", integral);

    proportional = error*kP;
    derivative = (error - last_error)*kD;
    integral = accError*kI;

    proportional_correction = correction_error*kP_correction;

    final_power = proportional + derivative;

    if(final_power > max_speed)
    {
    final_power = max_speed;
    }

    if(final_power < -max_speed)
    {
    final_power = -max_speed;
    }

    printf("final_power val %f\n", final_power);

     if (fabs(error) < (1.0/circ)* 900.0){
     timer_drive = false;
     }
     else{
     net_timer = pros::millis() + timeout;
     }

     last_error = error;


     if(forward_direction == true && fabs(correction_error) > 0.0)
     {

     leftdrive_set(final_power + (proportional_correction));
     rightdrive_set(final_power - (proportional_correction));
     }

     else if(forward_direction == false && fabs(correction_error) > 0.0)
     {
       leftdrive_set(final_power - (proportional_correction));
       rightdrive_set(final_power + (proportional_correction));
     }

     else{
     leftdrive_set(final_power);
     rightdrive_set(final_power);
    }

     pros::delay(10);
  }


pros::lcd::print(7,"Distance travled %f\n", ((LF_mtr.get_position() + RF_mtr.get_position() + RB_mtr.get_position() + LB_mtr.get_position())/286.47889)); //

drive_set(0);
}
