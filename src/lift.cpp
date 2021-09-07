#include "lift.hpp"
#include "main.h"
#include "motors.hpp"
#include "sensors.hpp"
#include "tilter.hpp"
#include "tasks.hpp"
#include "PID.hpp"
#include "angler.hpp"

//pros::Mutex lift_mutex;
//pros::Mutex off_mutex;

// Initialize PID control loops for the lift in auto and op control modes.
PID liftAUTO(0.187, 0.125, 0.0, 100, 127, 200, 3000);
PID liftDRIVER(0.2235, 0.465, 0.0, 100, 127, 200, 3000);

namespace lift{

  bool g_auton_flag;
  bool g_opc_flag;
  bool deployMacro;
  bool g_clearTray = false;
  bool g_readyToLift = false;
  std::uint32_t g_timeout;

  int g_target;

  heights::lift lift_state = heights::E_OFF;
  heightsAUTO::auton lift_stateAUTO = heightsAUTO::E_OFF;


  const int points [heights::E_NUM_OF_HEIGHTS] = { 0, 2575, 2950};
  const int auton_points [heightsAUTO::E_NUM_OF_HEIGHTS] = { 0, 1850, 2350, 3000};

  void autonLift(int height, std::uint32_t timeoutLIFT, std::uint32_t timeoutTILT){   
    tilter::setTarget(tilter::State_Machine::E_LIFT, timeoutTILT);
    lift::setTargetAutonAsync(height, timeoutLIFT);
  }

  void autonLiftOff(int height, std::uint32_t timeoutLIFT, std::uint32_t timeoutTILT){   
    tilter::setTarget(tilter::State_Machine::E_OFF);
  }
  
  void deploy(){
    deployMacro = false;
  }

  void setTarget(int setTarget, std::uint32_t failsafe){

    switch(setTarget){

      case(heights::E_OFF):

        g_target = points[heights::E_OFF];
        lift_state = heights::E_OFF;
        break;

      case(heights::E_LOW):

        g_target = points[heights::E_LOW];
        lift_state = heights::E_LOW;
        g_clearTray = true;
        g_timeout = pros::millis() + 700;
        break;

      case(heights::E_MED):

        g_target = points[heights::E_MED];
        lift_state = heights::E_MED;
        g_clearTray = true;
        g_timeout = pros::millis() + 700;
        break;

    }

    liftDRIVER.setMaxVel(127);
    liftDRIVER.setInvoke(false);
    liftDRIVER.setSettle(250);
    liftDRIVER.setThreshold(200);
    liftDRIVER.calculateFailsafe(failsafe);

  }

  void setTargetAutonAsync(int setTarget, std::uint32_t failsafe){

    switch(setTarget){

      case(heightsAUTO::E_OFF):

        g_target = auton_points[heightsAUTO::E_OFF];
        lift_stateAUTO = heightsAUTO::E_OFF;
        break;

      case(heightsAUTO::E_CUBES):

        g_target = auton_points[heightsAUTO::E_CUBES];
        lift_stateAUTO = heightsAUTO::E_CUBES;
        break;

      case(heightsAUTO::E_LOW):

        g_target = auton_points[heightsAUTO::E_LOW];
        lift_stateAUTO = heightsAUTO::E_LOW;
        break;

      case(heightsAUTO::E_MED):

        g_target = auton_points[heightsAUTO::E_MED];
        lift_stateAUTO = heightsAUTO::E_MED;
        break;

    }

    liftAUTO.setMaxVel(127);
    liftAUTO.setInvoke(false);
    liftAUTO.setSettle(250);
    liftAUTO.setThreshold(200);
    liftAUTO.calculateFailsafe(failsafe);

  }


  void setTargetAuton(int setTarget, std::uint32_t failsafe){

    switch(setTarget){

      case(heightsAUTO::E_OFF):

        g_target = auton_points[heightsAUTO::E_OFF];
        lift_stateAUTO = heightsAUTO::E_OFF;
        break;

      case(heightsAUTO::E_CUBES):

        g_target = auton_points[heightsAUTO::E_CUBES];
        lift_stateAUTO = heightsAUTO::E_CUBES;
        break;


      case(heightsAUTO::E_LOW):

        g_target = auton_points[heightsAUTO::E_LOW];
        lift_stateAUTO = heightsAUTO::E_LOW;
        break;

      case(heightsAUTO::E_MED):

        g_target = auton_points[heightsAUTO::E_MED];
        lift_stateAUTO = heightsAUTO::E_MED;
        break;

      std::uint32_t initial_time = pros::millis();

      liftAUTO.setMaxVel(120);
      liftAUTO.setInvoke(false);
      liftAUTO.setSettle(250);
      liftAUTO.setThreshold(200);
      liftAUTO.calculateFailsafe(failsafe);

      while(pros::millis() < liftAUTO.getFailsafe()){ 

        float final_power = liftAUTO.calculate(g_target, lift_mtr.get_position());
        lift_set(final_power);

        liftAUTO.calculateTimer(liftAUTO.getSettle(), liftAUTO.getError());

        pros::delay(10);
      }
    }

  }


  heights::lift getTarget(){
    return lift_state;
  }



  void lift_task(void* param){

    okapi::ControllerButton l1(okapi::ControllerDigital::L1);
  	okapi::ControllerButton l2(okapi::ControllerDigital::L2);
    okapi::ControllerButton up(okapi::ControllerDigital::up);

    while(true){

      while(pros::competition::is_autonomous()){

        float final_power = liftAUTO.calculate(g_target, lift_mtr.get_position());

        if(pros::millis() < liftAUTO.getFailsafe()){ 
          lift_mtr.move(final_power);
        }
        else{
          lift_mtr.move_absolute(0.0, 200);
        }

        pros::delay(10);
      }

      while(!pros::competition::is_autonomous()){
        float final_power = 0.0;

        if(lift_mtr.get_position() > 500 && lift_mtr.get_position() < 1500){
            lift_state = heights::E_LOW;
        }
        else if(tilter_mtr.get_position() > 1501){
          lift_state = heights::E_MED;
        }
        else { lift_state = heights::E_OFF; }


        if(l1.changedToPressed()){ 
          lift::setTarget(heights::E_MED, 3000);
          pros::delay(500);

          std::cout << "LIFT position " << lift_mtr.get_position() << std::endl << std::endl;
        }

        if(l2.changedToPressed()){
          lift_mtr.move_absolute(0, 200);
          pros::delay(1500);
        }

        if(up.changedToPressed()){
          lift::setTarget(heights::E_LOW, 2500);
          pros::delay(500);
        }

        if(master_controller.get_digital(DIGITAL_DOWN)){
          tilter::setTarget(tilter::State_Machine::E_LIFT);
          pros::delay(20);

          lift_mtr.move_absolute(3000, 200);
          pros::delay(1000);

          lift_mtr.move_absolute(0, 200);
          pros::delay(1300);
              
          tilter::setTarget(tilter::State_Machine::E_OFF);
        }

        final_power = liftDRIVER.calculate(g_target, lift_mtr.get_position());

        if(pros::millis() < liftDRIVER.getFailsafe() && tilter::g_liftIsReady && lift::g_readyToLift == true){
          lift_set(final_power);
        }
        else {
          lift_set(0);
          lift_mtr.move_velocity(0);
        }

        std::cout << "LIFT position " << lift_mtr.get_position() << std::endl << std::endl;
        pros::delay(10);
      }

      pros::delay(10);
    }
  }




}
