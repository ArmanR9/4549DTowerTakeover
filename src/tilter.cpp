#include "tilter.hpp"
#include "main.h"
#include "motors.hpp"
#include "sensors.hpp"
#include "lift.hpp"
#include "utilities.hpp"
#include "tasks.hpp"
#include "pidd.hpp"

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
      pid_values angler_pid(0.25, 0.4, 0.75, 30, 500, 127);
     int iterator2;
     okapi::ControllerButton b(okapi::ControllerDigital::B);
     okapi::ControllerButton a(okapi::ControllerDigital::A);
     okapi::ControllerButton l1(okapi::ControllerDigital::L1);
     okapi::ControllerButton l2(okapi::ControllerDigital::L2);
     okapi::ControllerButton up(okapi::ControllerDigital::up);
     okapi::ControllerButton x(okapi::ControllerDigital::X);
     okapi::ControllerButton y(okapi::ControllerDigital::Y);

      bool computeTorque;

      float error;
      float last_error;

      float p, i, d;
      float integral_active_zone = 500.0;
      float integral_limit;

      int final_power;

      float kP = 0.25; // 0.03
      float kD = 0.2; // 0.6
      float kI = 1.0;

      float kP_a = 0.2;
      float kD_a = 0.275;
      float kI_a = 0.5;

      float position{0.0};
      float last_position{0.0};

      int max_time = 5000;
      int threshold = 200;
      int settle = 400;
      uint32_t timer = pros::millis();
      uint32_t _last_time{0};
      bool invoke_timer;
      uint32_t failsafe = pros::millis();

      state = State_Machine::E_OFF;

    while(true){

      while(pros::competition::is_autonomous()){
        failsafe = g_failsafe;

        position = tilter_mtr.get_position();

        error = g_target - position;
        p = kP * error;
        i += kI * error;
        d = kD * (position - last_position);

        if(fabs(error)){ // 500
        i = 0.0;
        }

        else if(fabs(error) > 300){
        i = 0.0;
        }


        if(fabs(i) > 30){
        i = 30 * sgn_(i);
        }

        


        final_power = pid_calc(&angler_pid, g_target, position);//p + i + d;


        if(fabs(final_power) > 110){
        final_power = 110 * sgn_(final_power);
      }

        if(fabs(angler_pid.error) < 500){
        final_power = final_power * 0.75;
        }

    
      if(fabs(error) < threshold){
       invoke_timer = true;
       isDone = true;
      }

      else{ invoke_timer = false;
      isDone = false;
      }

      if(!invoke_timer){ timer = pros::millis() + settle;}

        if( pros::millis() < failsafe){
        tilter_set(final_power);
        }

        else if(fabs(error) <= 10){
          tilter_set(0);
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

          if(y.changedToPressed()){
          tilter::setTarget(tilter::State_Machine::E_LIFT, 120000);
          }

          if(x.isPressed()){
          g_manualStack = true;
        //  tilter_mtr.move(60);
          }

          else{
          g_manualStack = false;
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
        i += kI * error;
        d = kD * (position - last_position);

       
       if(fabs(error) > 300){
        i = 0.0;
        }

        if(fabs(error) < 5){
        i = 0.0;
        }


        if(fabs(i) > 30){
        i = 30 * sgn_(i);
        }


        final_power = pid_calc(&angler_pid, g_target, position);// p + i + d;


     //   if(fabs(final_power) > 120){
    //    final_power = 120 * sgn_(final_power);
    //  }

      if(fabs(error) < threshold){
       invoke_timer = true;
      }

      else{ invoke_timer = false;}

      if(!invoke_timer){ timer = pros::millis() + settle;}

        if(g_manualStack){
        final_power = 60;
        }

        if(pros::millis() > g_t_minTime && pros::millis() < g_t_maxTime && g_torqueLoop){
            computeTorque = true;
          }
          else { computeTorque = false; }


          if (fabs(angler_pid.error) < 1300) {
            if (angler_pid.max_power < 120 * 0.65) {
              angler_pid.max_power = 120 * 0.65;
            } else {
              angler_pid.max_power = angler_pid.max_power - 15;
            }
          } else {
            angler_pid.max_power = 120;
          }

         //   if(tilter_mtr.get_torque() > 1.50 && computeTorque){
           // kP = 0.0;
           // kD = 0.0;
            // kI = 0.0035;
      // / /   // g_torqueLoop = false;
         //  }

        //   else{
          // kI = 0.0028;
        //   }

        //    else if(!computeTorque){
          //     kP = 0.0735; // 0.03
          //     kD = 0.077;
          //     g_torqueLoop = false;
          //  }

        std::cout << "error" << error << std::endl << std::endl;
        std::cout << "p " << p << std::endl << std::endl;
        std::cout << "i " << i << std::endl << std::endl;
        std::cout << "d "  << d << std::endl << std::endl;
        std::cout << "TILT "  << tilter_mtr.get_position() << std::endl << std::endl;


        if((pros::millis() < g_failsafe) || g_manualStack){ //&& g_readyToStack){
        tilter_set(final_power);
        }

        else if(fabs(error) <= 10){
        tilter_set(0);
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
