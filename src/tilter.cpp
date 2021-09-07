#include "tilter.hpp"
#include "main.h"
#include "motors.hpp"
#include "sensors.hpp"
#include "lift.hpp"
#include "utilities.hpp"
#include "tasks.hpp"
#include "PID.hpp"

namespace tilter{

  bool g_auton_flag;
  bool g_opc_flag;
  bool isDone = false;

  State_Machine state;
  int g_target = 0;
  int g_max_time;
  int g_settle;
  int g_failsafe = 0;

  bool g_clearTray = false;
  bool g_readyToStack = true;
  bool g_liftIsReady = false;

  bool g_torqueLoop = false;

  std::uint32_t g_t_minTime;
  std::uint32_t g_t_maxTime;

  std::uint32_t g_timeout;

  bool g_manualStack;

  void setTarget(State_Machine setTarget, std::uint32_t ifailsafe){
      g_failsafe = pros::millis() + ifailsafe;

      switch(setTarget){

        case(State_Machine::E_OFF):
          g_target = -400;
          break;

        case(State_Machine::E_STACK):
          g_target = 4850;
          g_clearTray = true;
          g_timeout = pros::millis() + 1000;
          g_t_minTime = pros::millis() + 650;
          g_t_maxTime = pros::millis() + 850;
          g_torqueLoop = true;
          break;

        case(State_Machine::E_LIFT):
          g_target = 2000;
          break;
      }

  }

  State_Machine get_target(){
    return state;
  }

  bool isSettled(){
    if(tilter_mtr.get_position() > 4750){
      pros::delay(500);
      return true;
    }
    else{
      return false;
    }
  }

  void waitUntilSettled(uint32_t timeout){
    while(!isSettled()){
      pros::delay(timeout);
      break; 
    }
  }


  void tilter_task(void * param){

    PID tilterPID(0.2, 0.75, 0.76, 200, 127, 400, 5000);

    okapi::ControllerButton b(okapi::ControllerDigital::B);
    okapi::ControllerButton a(okapi::ControllerDigital::A);
    okapi::ControllerButton l1(okapi::ControllerDigital::L1);
    okapi::ControllerButton l2(okapi::ControllerDigital::L2);
    okapi::ControllerButton up(okapi::ControllerDigital::up);
    okapi::ControllerButton x(okapi::ControllerDigital::X);
    okapi::ControllerButton y(okapi::ControllerDigital::Y);

    float final_power{0.0};
    float position{0.0};
    
    
    uint32_t failsafe = pros::millis();

    state = State_Machine::E_OFF;

    while(true){

      while(pros::competition::is_autonomous()){
        failsafe = g_failsafe;

        position = tilter_mtr.get_position();

        final_power = tilterPID.calculate(g_target, position);

        if(pros::millis() < tilterPID.getFailsafe()){ 
          tilter_set(final_power);
        }
        else{
          tilter_mtr.move_absolute(0.0, 200);
        }

        pros::delay(10);

      }

      while(!pros::competition::is_autonomous()){

        if(tilter_mtr.get_position() > 1000 && tilter_mtr.get_position() < 4000){
          state = State_Machine::E_LIFT;
          g_liftIsReady = true;
        }
        else if(tilter_mtr.get_position() > 4001){
          state = State_Machine::E_STACK;
        }
        else { state = State_Machine::E_OFF;
          g_liftIsReady = false;
        }

        if(a.changedToPressed()){
          tilter::setTarget(tilter::State_Machine::E_STACK);
        }

        if(b.changedToPressed()){
          tilter::setTarget(tilter::State_Machine::E_OFF);
        }

        if(y.changedToPressed()){
          tilter::setTarget(tilter::State_Machine::E_LIFT, 120000);
        }

        if(x.isPressed()){
          g_manualStack = true;
        }

        else{
          g_manualStack = false;
        }


        if(l1.changedToPressed()){
          tilter::setTarget(tilter::State_Machine::E_LIFT);
        }

        if(l2.changedToPressed()){
          tilter::setTarget(tilter::State_Machine::E_OFF);
        }

        if(up.changedToPressed()){
          tilter::setTarget(tilter::State_Machine::E_LIFT);
        }

        final_power = tilterPID.calculate(g_target, position);

        if(pros::millis() < tilterPID.getFailsafe()){
          tilter_set(final_power);
        }

        else {
          tilter_set(0);
          tilter_mtr.move_velocity(0);
        }

        if(g_manualStack){
          final_power = 60;
        }

        tilterPID.logTimer();

        pros::delay(10);
      }

    pros::delay(10);
    }
  }


} 