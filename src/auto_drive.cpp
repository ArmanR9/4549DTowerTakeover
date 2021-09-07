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

void driveToPosition(float y, float x, float ys, float xs, float maxErrX, float maxVel, std::uint32_t ifailsafe, bool enableCorrect, bool forward, bool harshStop){
 
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

  do{

          pos_drive.calculateTimer(pos_drive.getSettle(), pos_drive.getError());

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

          if(direction == _Dir::FWD){
            final_power = pos_drive.calculate(targetVar, cur_pos_vector.y);
          }
          else if (direction == _Dir::BWD){
            final_power = -1 * pos_drive.calculate(targetVar, cur_pos_vector.y);
          }

          switch(sgn_(correction)){

            case(1):
              driveLR_set(final_power, final_power * exp(-correction));
              break;

            case(-1):
              driveLR_set(final_power * exp(correction), final_power);
              break;

            case(0):
              driveLR_set(final_power, final_power);
              break;
          }


           std::cout << "Cur Pos Vec Y: " << cur_pos_vector.y << std::endl << std::endl;
           std::cout << "target var" << targetVar << std::endl << std::endl;
           std::cout << "final_power" << final_power << std::endl << std::endl;
           std::cout << "correction" << correction << std::endl << std::endl;

           pros::delay(10);

  } while(cur_pos_vector.y < 0.0 && pros::millis() < pos_drive.getFailsafe()); //&& pros::millis() < pos_drive.getTimer());

  driveLR_set(0, 0);
}


void driveToDistance(float d, float a, float ys, float xs, float maxErrX, float maxVel, std::uint32_t ifailsafe, bool enableCorrect, bool forward, bool harshStop){
// Calculate the x and y positons using the hypotenuse and angle of the global ending triangle
driveToPosition(ys + d * cos(a), xs + d * sin(a), ys, xs, maxErrX, maxVel, ifailsafe, enableCorrect, forward, harshStop);
}

void turn2ang(float angle, int max_velocity, _TurnDir direction, uint32_t settle, uint32_t max_time, float ikI){

  float error;
  float final_error = 0.0;
  float last_error = 0.0;

  float proportional, integral, derivative;

  float final_power;

  float threshold = 0.25;

  std::uint32_t failsafe = pros::millis() + max_time;
  uint32_t timer = pros::millis() + settle;
  bool invoke_timer = false;

  if(direction == _TurnDir::CH){
    if(fmod(angle - pos.get_alpha(), PI * 2) > PI) {
      direction = _TurnDir::CCW;
    } else { direction = _TurnDir::CW; }
  }

  float kP = 70.05;
  float kD = 172.90;
  float kI = ikI;

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

    while(pros::millis() < failsafe && pros::millis() < timer){

      if(direction == _TurnDir::CW){        
        final_error = error - pos.get_alpha();
      }
      else if(direction == _TurnDir::CCW){
        final_error = pos.get_alpha() - error;
      }       
      
      proportional = kP * final_error;
      derivative = kD * (final_error - last_error);
      integral += (kI * final_error);

      if(fabs(radians_to_degrees(final_error)) > 5.75 || fabs(radians_to_degrees(final_error)) < 1.85){
        integral = 0.0;
      }
      else {integral += (kI * final_error); }

      if(fabs(integral) > 50){
        integral = 50 * sgn_(integral);
      }

      if(fabs(radians_to_degrees(final_error)) < threshold){
         invoke_timer = true;
      }
      else if (!invoke_timer){
        timer = pros::millis() + settle;
      }

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

              
      std::cout << "nearest angle " << radians_to_degrees(error) << std::endl << std::endl;
      std::cout << " error " << radians_to_degrees(final_error) << std::endl << std::endl;
      std::cout << "ALPHA " << radians_to_degrees(pos.get_alpha()) << std::endl << std::endl;
      std::cout << "INTEGRAL " << integral << std::endl << std::endl;
     

      last_error = final_error;


      pros::delay(10);
    }

  leftdrive_set(0);
  rightdrive_set(0);
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

  isAccel_done = true; // Acceleration is done, enable flag and start normal motion algorithim.

// send target as global, calculate the error during this function, do target -= target - error to find remaining target left, and feed it into drive_encoder
}

void drive_lineup(int voltage, uint32_t give_time){
  drive_set(voltage);
  pros::delay(give_time);
  drive_set(0);
}

void turning_lineup(int voltage, uint32_t give_time){
  turning_set(voltage);
  pros::delay(give_time);
  turning_set(0);
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
