#include "tilter.hpp"
#include "main.h"
#include "motors.hpp"
#include "sensors.hpp"
#include "lift.hpp"
#include "utilities.hpp"
#include "tasks.hpp"

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
  std::uint32_t g_timeout;

  void setTarget(State_Machine setTarget){
      g_failsafe = pros::millis() + 10000;

      switch(setTarget){

            case(State_Machine::E_OFF):

                g_target = -200;
                break;

                case(State_Machine::E_STACK):

                g_target = 4900;
                g_clearTray = true;
                g_timeout = pros::millis() + 1000;
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
    else
    return false;
   }

   void waitUntilSettled(uint32_t timeout){
    while(!isSettled()){
    pros::delay(timeout);
    break;
    }
   }




  void tilter_task(void * param){
     int iterator2;
     okapi::ControllerButton b(okapi::ControllerDigital::B);
     okapi::ControllerButton a(okapi::ControllerDigital::A);
     okapi::ControllerButton l1(okapi::ControllerDigital::L1);
     okapi::ControllerButton l2(okapi::ControllerDigital::L2);
     okapi::ControllerButton up(okapi::ControllerDigital::up);

      float error;
      float last_error;

      float p, i, d;
      float integral_active_zone = 500.0;
      float integral_limit;

      float final_power;

      float kP = 0.07425; // 0.03
      float kD = 0.07715; // 0.6
      float kI = 0.0;

      float kP_a = 0.07425;
      float kD_a = 0.07715;
      float kI_a = 0.0;

      float position{0.0};
      float last_position{0.0};

      int max_time = 5000;
      int threshold = 250;
      int settle = 100;
      uint32_t timer = pros::millis();
      uint32_t _last_time{0};
      bool invoke_timer;
      uint32_t failsafe = pros::millis();

      state = State_Machine::E_OFF;

    while(true){

      while(pros::competition::is_autonomous()){
      //  std::cout << i << std::endl;
        failsafe = g_failsafe;

        position = tilter_mtr.get_position();

        error = g_target - position;
        p = kP_a * error;
        i += kI_a * error;
        d = kD_a * (position - last_position);

        if(fabs(error) < threshold){ // 500
        i = 0.0;
        }

        if(fabs(error) > 2000){
        i = 0.0;
        }

        if(fabs(error) > 50){
        i = 0;
        }

        if(fabs(i) > 50){
        i = 50 * sgn_(i);
        }


        final_power = p + d + i; //+ d;


        if(fabs(final_power) > 110){
        final_power = 110 * sgn_(final_power);
      }

      if(fabs(error) < threshold){
       invoke_timer = true;
       isDone = true;
      }

      else{ invoke_timer = false;
      isDone = false;
      }

      if(!invoke_timer){ timer = pros::millis() + settle;}

        if(pros::millis() < timer && pros::millis() < failsafe){
        tilter_set(final_power);
        }


      else {
        tilter_set(0);
      }



        last_error = error;
        last_position = position;

        pros::delay(10);
      }

      while(!pros::competition::is_autonomous()){// && g_opc_flag){

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


          if(l1.changedToPressed()){
            tilter::setTarget(tilter::State_Machine::E_LIFT);
          //  pros::delay(1000);
        //    lift_mtr.move_absolute(2750, 200);
          }

          if(l2.changedToPressed()){
          //  lift_mtr.move_absolute(0, 200);
          //  pros::delay(1000);
          pros::delay(500);
            tilter::setTarget(tilter::State_Machine::E_OFF);
      //      pros::delay(500);
        //    g_liftIsReady = false;
          }

          if(up.changedToPressed()){
            tilter::setTarget(tilter::State_Machine::E_LIFT);
          //  tilter_mtr.move_absolute(2000, 100);
            pros::delay(1000);
            g_liftIsReady = true;

          }

      /*    if(l1.changedToPressed()){
                  lift::setTarget(lift::heights::E_MED);
                  tilter::setTarget(tilter::State_Machine::E_LIFT);
                  pros::delay(500);
                  lift_task_ctor->notify();
          }
          */
      /*  if(a.changedToPressed() && iterator2 < 2){
        iterator2++;
        tilter::setTarget(tilter::State_Machine::E_STACK);
        }

        if(b.changedToPressed() && iterator2 > 0){
        	iterator2 --;
        tilter::setTarget(tilter::State_Machine::E_OFF);
        }
*/


      //if(g_target == 2000){// && tilter_task_ctor->notify_take(true, TIMEOUT_MAX)){
      //runPID
    //  pros::delay(1);
  //    lift_task_ctor->notify();
  //    tilter_task_ctor->notify_clear();
    //  }

    //  else if(g_target == 0 && lift::getTarget() == 0){ //&& tilter_task_ctor->notify_take(true, TIMEOUT_MAX)){
      //runPID
    //  lift_task_ctor->notify_clear();
    //  tilter_task_ctor->notify_clear();
     // }
/*
       if(g_target == 2000){

        lift_mutex.take(9999);
        tilter_mtr.move_absolute(2000, 75);
        pros::delay(500);
        tilter::setTarget(State_Machine::E_LIFT);
        lift_mutex.give();
      }
      */
/*
      else if(g_target == 0 && lift::getTarget() == 0){
      lift_mutex.take(9999);
      tilter_mtr.move_absolute(0, 90);
      pros::delay(2000);
      tilter::setTarget(State_Machine::E_OFF);
      lift_mutex.give();
      }
*/

   //   else {//if(g_target == 5000 || g_target == 0){

        failsafe = g_failsafe;

        position = tilter_mtr.get_position();

        error = g_target - position;
        p = kP * error;
        i += error;
        d = kD * (position - last_position);

        if(fabs(error) < threshold){ // 500
        i = 0;
        }

        if(fabs(error) < 50){
        i = 0;
        }

        if(fabs(i) > 50){
        i = 50 * sgn_(i);
        }


        final_power = p + i + d;


        if(fabs(final_power) > 110){
        final_power = 110 * sgn_(final_power);
      }

      if(fabs(error) < threshold){
       invoke_timer = true;
      }

      else{ invoke_timer = false;}

      if(!invoke_timer){ timer = pros::millis() + settle;}

        if(pros::millis() < g_failsafe && g_readyToStack){
        tilter_set(final_power);
        }

      else { tilter_set(0); }



  //    if(light_sensor.get_value() > light_sensor_threshold){// && g_clearTray){ //&& pros::millis() < g_timeout){
  //    intake_set(-80);
    //  }
    //  else { intake_set(0);
  //    g_clearTray = false;
  //    }
//
        last_error = error;
        last_position = position;
        // tilter_mtr.move_absolute(g_target, 75);
       //}

        //std::cout << "l1" << l1.changedToPressed() << std::endl << std::endl;
      //  std::cout << "l2" << l2.changedToPressed() << std::endl << std::endl;

        pros::delay(10);
      }

    pros::delay(10);
    }
  }




}







/*
















void Tilter::set_state(State_Machine s){
m_targetState = s;
computeTarget(s);
}

void Tilter::computeTarget(State_Machine &target_state){

  switch (target_state){

    case(State_Machine::E_OFF):

      m_target = m_angles[E_OFF_A];
      break;

     case(State_Machine::E_STACK):

      m_target = m_angles[E_STACK_A];
      break;


     case(State_Machine::E_LIFT):
     m_target = m_angles[E_LIFT_A];
     break;
   }
}




void TilterPID::reset() {
  m_error = 0;
  m_lastError = 0;
  m_proportional = 0;
  m_derivative = 0;
  m_final_power = 0;
}
*/
