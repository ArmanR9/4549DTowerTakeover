#include "auto_drive.hpp"
#include "main.h"
#include "motors.hpp"
#include "sensors.hpp"
#include "utilities.hpp"
#include "odometry.hpp"
#include "PID.hpp"

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

//double targetA = lineAngle(targetX, targetY, startX, startY);

while(pros::millis() < timer && pros::competition::is_autonomous() && pros::millis() < failsafe){
currVector_x += pos.get_x() - startX;
currVector_y += pos.get_y() - startY;

 _currVector.x = pos.get_x() - targetX;
 _currVector.y = pos.get_y() - targetY;
 _currPolar = vector_to_polar(_currVector, _currPolar);
 _currPolar.theta += 0; //targetA;
 _currVector = polar_to_vector(_currPolar, _currVector);



//currX += pos.x;
//currY += pos.y;

errorA = 0;//targetA - pos.get_alpha();
errorX = 0;//targetX - currVector_x;
errorY = 0;//targetY - currVector_y;

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



void straight_line_drive(float setpoint, float angle, float kP_correction, float maxErrA, int max_velocity, uint32_t settle, uint32_t max_time){

      double encL_initial = encoder360L.get_value();
      double encR_initial = encoder360R.get_value();

      double encL;
      double encR;

      double c_error;
      double c_error2;
      double error;
      float last_error;

      float encoder_average;
      float proportional, integral, derivative;
      float p_correction;


      float final_power;

      bool integralActiveZone;
      float IntegralLimit = 50.0;

      float position{0.0};
      float last_position{0.0};

      float threshold = 1.0;
      uint32_t timer = pros::millis() + settle;
      bool invoke_timer = true;
      uint32_t failsafe = pros::millis() + max_time;

      uint32_t dT{0};
      uint32_t _last_time{0};

      bool forward_direction;




            if (setpoint > 0.0) {
              forward_direction = true;
              }
              else if (setpoint < 0.0) {
              forward_direction = false;
              }


              if(angle == 'n'){
              angle = radians_to_degrees(pos.get_alpha());
              }


   float kP = 5.5; //0.301
 //  float kP_correction = 150.0;
   float kD = 6.5;
   float kI = 0.0;

  while(pros::millis() < timer && pros::millis() < failsafe) {  // ((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis()))

      encL = encoder360L.get_value() - encL_initial;
      encR = encoder360R.get_value() - encR_initial;
      encL *= ticks_to_in;
      encR *= ticks_to_in;

     encoder_average = (encL + encR)/2.0;
     position = encoder_average;

     c_error = degrees_to_radians(angle) - pos.get_alpha();
     c_error2 = atan2(sin(c_error), cos(c_error));

     error = setpoint - encoder_average;





     proportional = error * kP;
     derivative = (position - last_position)*kD;
 //   integral = accError*kI;

      p_correction = c_error2 * kP_correction;

      final_power = proportional + derivative;




      if(fabs(final_power) > max_velocity){
        final_power = max_velocity * sgn_(final_power);
      }



      if(fabs(error) < threshold){
       invoke_timer = false;
      }

      else if(invoke_timer){
        timer = pros::millis() + settle;
      }
        // invoke_timer = false;}

    //  if(!invoke_timer){ timer = pros::millis() + settle;}



     last_error = error;
     last_position = position;


     if(forward_direction == true && fabs(radians_to_degrees(c_error2)) > maxErrA)
     {

     leftdrive_set(final_power + (p_correction));
     rightdrive_set(final_power - (p_correction));
     }

     else if(forward_direction == false && fabs(radians_to_degrees(c_error2)) > maxErrA)
     {
       leftdrive_set(final_power - (p_correction));
       rightdrive_set(final_power + (p_correction));
     }

     else{
     leftdrive_set(final_power);
     rightdrive_set(final_power);
      }


     std::cout << "global angle" << pos.get_alpha() << std::endl;

     std::cout << "encL" << encL << std::endl;

     std::cout << "encR" << encR << std::endl;

     std::cout << "c error" << c_error << std::endl;

     std::cout << "c error2" << c_error2 << std::endl;

     pros::delay(10);
  }


drive_set(0);
}





void turn2ang(float angle, int max_velocity, _TurnDir direction, uint32_t settle, uint32_t max_time){

  float error;
  float final_error = 0.0;

  float last_error = 0.0;

  float proportional, integral, derivative;

  float final_power;

  float threshold = 0.25;

  std::uint32_t failsafe = pros::millis() + max_time;
  uint32_t timer = pros::millis() + settle;
  bool invoke_timer = false;

 // _TurnDir direction;

   if(direction == _TurnDir::CH){

     if(fmod(angle - pos.get_alpha(), PI * 2) > PI) {
       direction = _TurnDir::CCW;
      } else { direction = _TurnDir::CW; }

   }

      float kP = 86.55;
      float kD = 105.75;
      float kI = 2.789;

      float kPcube = 82.25;
      float kDcube = 97.25;
      float kIcube = 0.0;


      switch(direction){

        case (_TurnDir::CW):
        error = pos.get_alpha() + fmod(degrees_to_radians(angle) - pos.get_alpha(), M_PI * 2);
        break;

        case(_TurnDir::CCW):
        error = pos.get_alpha() - fmod(degrees_to_radians(angle) - pos.get_alpha(), M_PI * 2);
        break;

      }

      //error = pos.get_alpha() + fmod(degrees_to_radians(angle) - pos.get_alpha(), M_PI * 2);

      while(pros::millis() < failsafe && pros::millis() < timer){//pros::millis() < timer && pros::millis() < failsafe){

        //error = degrees_to_radians(angle) - pos.get_alpha();
       // final_error = atan2(sin(error), cos(error));
      //  error = pos.get_alpha() + _fmod(pos.get_alpha() - degrees_to_radians(angle), (M_PI * 2));
        if(direction == _TurnDir::CW){        
        final_error = error - pos.get_alpha();
        }
        else if(direction == _TurnDir::CCW){
        final_error = pos.get_alpha() - error;
        }       //  atan2(sin(error), cos(error));      //  error - pos.get_alpha();
        

     
        proportional = kP * final_error;
        derivative = kD * (final_error - last_error);
      //  integral += (kI * final_error);

       if(fabs(radians_to_degrees(final_error)) > 6.75 || fabs(radians_to_degrees(final_error)) < 0.45){
       integral = 0.0;
        }
        else {integral += (kI * final_error); }

        if(fabs(integral) > 50){
        integral = 50 * sgn_(integral);
        }



        if(fabs(radians_to_degrees(final_error)) < threshold){
         invoke_timer = true;
       //  integral = 0.0;
        }

        else if (!invoke_timer){
      //  invoke_timer = false;
        timer = pros::millis() + settle;
        }

        //if(!invoke_timer){ timer = pros::millis() + settle;}



        final_power = proportional + derivative + integral;


        if(final_power > max_velocity){
        final_power = max_velocity * sgn_(final_power);
        }


          if(direction == _TurnDir::CW){
              leftdrive_set(final_power);
              rightdrive_set(-final_power);
            }

          else if(direction == _TurnDir::CCW){
                leftdrive_set(-final_power);
                rightdrive_set(final_power);
              }

              //std::cout << "direction" << static_cast<int>(direction) << std::endl;
              std::cout << "nearest angle " << radians_to_degrees(error) << std::endl << std::endl;
              std::cout << " error " << radians_to_degrees(final_error) << std::endl << std::endl;
              std::cout << "ALPHA " << radians_to_degrees(pos.get_alpha()) << std::endl << std::endl;
              std::cout << "INTEGRAL " << integral << std::endl << std::endl;
           //   std::cout << "final_power" << final_power << std::endl;
         //     std::cout << "pros::millis" << pros::millis() << std::endl;
          //    std::cout << "timer" << timer << std::endl;


        last_error = final_error;


        pros::delay(10);
      }

leftdrive_set(0);
rightdrive_set(0);
}


// double kP, double kD, double kI, double threshold, double maxVel, uint32_t settle, uint32_t max_time
void driveToPosition(float y, float x, float ys, float xs, float maxErrX, float maxVel, std::uint32_t ifailsafe, bool enableCorrect, bool forward, bool harshStop){

// Initialize PID
  PID pos_drive(3.25, 2.05, 0.0, 2.0, 100, 200, 5000);

// Our current x/y within the motion alg
  Vector cur_pos_vector;
  Polar cur_pos_polar;

// Path line
  Path followLine;

  followLine.p1.x = xs;
  followLine.p1.y = ys;

  followLine.p2.x = x;
  followLine.p2.y = y;

  // Length of the line
  float lineLength = getLineLength(followLine);
  // The angle of the line to rotate our local position to.
  float lineAngle = getLineAngle(followLine);
  // nearest angle to the line (Flip the angle by 180 degrees if moving backwards)
  float pidAngle = nearAngle((lineAngle - (forward == false ? M_PI : 0)), pos.get_alpha());

  pos_drive.setMaxVel(maxVel);
  pos_drive.setInvoke(false);
  pos_drive.setSettle(250);
  pos_drive.setThreshold(2.0);
  pos_drive.calculateFailsafe(ifailsafe);

  //Correction constant
  float kP_c = 5.0;

  // Correction value
  float correction = 0.0;

  float errorA = 0.0;
  float errorX = 0.0;
  float correctA = 0.0;

  float targetVar = 0.0;

  float final_power;

   _Dir direction = forward == true ? _Dir::FWD : _Dir::BWD;



  //    if(radians_to_degrees(std::abs(correctA)) > 7.5){
      //  turn2ang(correctA, 100, _TurnDir::CH, 300, 5000);
      //  pros::delay(500);
    //  }

//      while(pros::millis() < pos_drive.getFailsafe()){


      //  std::cout << "LINE A: " << lineAngle  << std::endl << std::endl;


      //  pos_drive.calculateTimer(pos_drive.getSettle(), pos_drive.getError());

        do{
         // std::cout << "LINE A: " << lineAngle  << std::endl << std::endl;


          pos_drive.calculateTimer(pos_drive.getSettle(), pos_drive.getError());

         //  std::cout << "get x: " << pos.get_x()  << std::endl << std::endl;
         //  std::cout << "vector x: " << cur_pos_vector.x  << std::endl << std::endl;
         //  std::cout << "error x: " << errorX << std::endl << std::endl;


          cur_pos_vector.x = pos.get_x() - x;
          cur_pos_vector.y = pos.get_y() - y;
          vector_to_polar(cur_pos_vector, cur_pos_polar);
          cur_pos_polar.theta += lineAngle;
          polar_to_vector(cur_pos_polar, cur_pos_vector);

          if(enableCorrect){

           errorA = pos.get_alpha() - pidAngle;
           errorX = cur_pos_vector.x + cur_pos_vector.y * sin(errorA) / cos(errorA);
           correctA = atan2(x - pos.get_x(), y - pos.get_y());

            correction = std::abs(errorX) > maxErrX ? kP_c * (nearAngle(correctA, pos.get_alpha()) - pos.get_alpha()) : 0.0;//nearAngle(correctA, pos.get_alpha() - pos.get_alpha()) : 0.0;
            if(direction == _Dir::BWD){
             correctA += M_PI;
            }
          }

          if(direction == _Dir::FWD){
          targetVar = -1 * cur_pos_vector.y;
          }

          else if(direction == _Dir::BWD){ targetVar = -1 * cur_pos_vector.y; }

        if(direction == _Dir::FWD)
        final_power = pos_drive.calculate(targetVar, cur_pos_vector.y);

        else if (direction == _Dir::BWD)
        final_power = -1 * pos_drive.calculate(targetVar, cur_pos_vector.y);

       // direction = sgn_(final_power) > 0 ? _Dir::FWD : _Dir::BWD;





         switch(sgn_(correction)){

         case(1):
         driveLR_set(final_power, final_power * exp(-correction));
         /*
         if(direction == _Dir::FWD){
         driveLR_set(final_power * exp(correction), final_power);
       }
          else if(direction == _Dir::BWD){
           driveLR_set((-1*final_power), ((-1*(final_power * (exp(-correction))))));
         }
         */
         break;

         case(-1):
         driveLR_set(final_power * exp(correction), final_power);

        // if(direction == _Dir::FWD){
        // driveLR_set(final_power * exp(correction), final_power);
      //___int64_t_defined }
      //    else{
      //    driveLR_set(-final_power * (-1*exp(correction)), -final_power);
      //  }S
         break;

         case(0):
         driveLR_set(final_power, final_power);
         break;
            }

      //  pros::delay(10);

           std::cout << "Cur Pos Vec Y: " << cur_pos_vector.y << std::endl << std::endl;
           std::cout << "target var" << targetVar << std::endl << std::endl;
           std::cout << "final_power" << final_power << std::endl << std::endl;
           std::cout << "correction" << correction << std::endl << std::endl;
     //    std::cout << "Alpha in Rads: " << pos.get_alpha() << std::endl << std::endl;
       //  std::cout << "Correct A: " << correctA << std::endl << std::endl;
        //std::cout << "failsafe : " << pos_drive.getFailsafe() << std::endl << std::endl;
 //std::cout << "millis : " << pros::millis() << std::endl << std::endl;

         pros::delay(10);

       } while(cur_pos_vector.y < 0.0 && pros::millis() < pos_drive.getFailsafe()); //&& pros::millis() < pos_drive.getTimer());

  //    if(harshStop){
    //  applyHarshStop();
    //  }
    //   else{
      //   driveLR_set(0,0);
       //}

      //   std::cout << "final_power" << final_power << std::endl << std::endl;
          // std::cout << "cur pos vec y" << cur_pos_vector.y << std::endl << std::endl;
      //   std::cout << "correction" << final_power << std::endl << std::endl;
  //       std::cout << "OUTPUT: " << final_power << std::endl << std::endl;
      //   std::cout << "FAILSAFE: " << pos_drive.getFailsafe() << std::endl << std::endl;
    //     std::cout << "CORRECTION: " << correction << std::endl << std::endl;
    //     std::cout << "CORRECT A: " << correctA  << std::endl << std::endl;
    //     std::cout << "ERROX X: " << errorX  << std::endl << std::endl;
    //     std::cout << "ERROR A: " << errorA  << std::endl << std::endl;
    //     std::cout << "CURRENT X " << cur_pos_vector.x  << std::endl << std::endl;
         //pos_drive.logTimer();

      //   pros::delay(10);


  driveLR_set(0, 0);
}



void driveToDistance(float d, float a, float ys, float xs, float maxErrX, float maxVel, std::uint32_t ifailsafe, bool enableCorrect, bool forward, bool harshStop){

// Calculate the x and y positons using the hypotenuse and angle of the global ending triangle
driveToPosition(ys + d * cos(a), xs + d * sin(a), ys, xs, maxErrX, maxVel, ifailsafe, enableCorrect, forward, harshStop);

}




void position_sweep(double y, double x, double ys, double xs, bool forward){

  double robotTrack = 10.5;

  // Difference between starting and end positions
  double dX = x - xs;
  double dY = y - ys;

  //Lookahead distance to point
  double l = sqrt((dX * dX) + (dY * dY));

  // Radius of the Arc
  double r = (l*l) / (2*dX);

  // Angular velocity of the robot
  double w = M_PI/6;//pos.get_angularV();

  // Curvature = 1 / r
  double c = 2*dX/(l*l);

  // Target velocity for wheels after solving for curvature
  double targetV = w / c;

  // Left and Right wheel velocities
  float lVelocity = 0.0;
  float rVelocity = 0.0;

  // Left and Right wheel acceleration values
  float lAcceleration = 0.0;
  float rAcceleration = 0.0;

  // Feedforward and Feedback gains

  double kV = 15.5; // Feedforward velocity constant
  double kA = 20.25; // Feedforward acceleration constant

  double kP, kD; // Normal feedback proportional/derivative gains

  double lFeedforward; // feedforward output
  double lFeedback;

  double rFeedforward;
  double rFeedback;

  double lFinal_Power, rFinal_Power;

  std::uint32_t initial_time = pros::millis();
  std::uint32_t current_time = initial_time;
  std::uint32_t dT = 0.0;
  std::uint32_t last_time = current_time;

   _Dir direction = forward == true ? _Dir::FWD : _Dir::BWD;


      while(true){

        current_time = pros::millis() - initial_time;
        dT = current_time - last_time;



        dX = x - xs;
        dY = y - ys;

        c = 2*dX/(l*l);
        w = M_PI/10.0;//pos.get_angularV();
        targetV = w / c;



        double lMeasured_Vel = wheel_vel.get_vel_L();
        double rMeasured_Vel = wheel_vel.get_vel_R();
        if(w){
        lVelocity = targetV * (2 + c * robotTrack)/2;
        rVelocity = targetV * (2 - c * robotTrack)/2;
      }
      else{
        lVelocity =  (2 + c * robotTrack)/2;
        rVelocity =  (2 - c * robotTrack)/2;
      }

        if(dT){ // To prevent divide by zero error
        lAcceleration = lVelocity / dT;
        rAcceleration = rVelocity / dT;
      }

        lFeedforward = (kV * lVelocity) + (kA * lAcceleration);
        rFeedforward = (kV * rVelocity) + (kA * rAcceleration);

        lFeedback = kP * (lVelocity - lMeasured_Vel);
        rFeedback = kP * (rVelocity - rMeasured_Vel);

        lFinal_Power = lFeedforward;
        rFinal_Power = rFeedforward;



          switch(direction){

              case(_Dir::FWD):

              driveLR_vel_set(lFinal_Power, rFinal_Power);
              break;

              case(_Dir::BWD):

              driveLR_set(-lFinal_Power, -rFinal_Power);
              break;
        }


      std::cout << "final_pwr:  " <<lFinal_Power << std::endl << std::endl;
      std::cout << "other:  " << rFinal_Power << std::endl << std::endl;
      last_time = current_time;
      pros::delay(10);
    }


  driveLR_vel_set(0,0);
}


void driveDistance(){

}
























void drive_lineup(int voltage, uint32_t give_time){
drive_set(voltage);
pros::delay(give_time);
drive_set(0);
}

void while_drive(int voltage, uint32_t give_time){

    while(true){
        drive_set(voltage);
        pros::delay(give_time);
        }
}

void drive_outtake(int drive, int intake, uint32_t give_time){
drive_set(drive);
intake_set(intake);
pros::delay(give_time);
}

void outtake(int voltage, uint32_t give_time){
intake_set(voltage);
pros::delay(give_time);
}

void lineup_left(int voltage, uint32_t give_time){
leftdrive_set(voltage);
pros::delay(give_time);
}

void lineup_right(int voltage, uint32_t give_time){
leftdrive_set(-voltage * 0.5);
rightdrive_set(voltage);
pros::delay(give_time);
}

void drive_relative(double position, int voltage){

position = position / (4.125 * M_PI / 900.0);

LB_mtr.move_relative(position, voltage);
LF_mtr.move_relative(position, voltage);
RB_mtr.move_relative(position, voltage);
RF_mtr.move_relative(position, voltage);

}
