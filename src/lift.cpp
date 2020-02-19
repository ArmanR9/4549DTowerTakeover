#include "lift.hpp"
#include "main.h"
#include "motors.hpp"
#include "sensors.hpp"
#include "tilter.hpp"
#include "tasks.hpp"
#include "PID.hpp"

pros::Mutex lift_mutex;
pros::Mutex off_mutex;


PID liftAUTO(0.187, 0.125, 0.0, 100, 127, 200, 3000);
PID liftDRIVER(0.2235, 0.465, 0.0, 100, 127, 200, 3000);


//TilterPID tilt(0,0,0,0,0,0);
namespace lift{

  bool g_auton_flag;
  bool g_opc_flag;
  bool deployMacro;
  bool g_clearTray = false;
  bool g_readyToLift = true;
  std::uint32_t g_timeout;

  int g_target;

  heights::lift lift_state = heights::E_OFF;
  heightsAUTO::auton lift_stateAUTO = heightsAUTO::E_OFF;


const int points [heights::E_NUM_OF_HEIGHTS] = { 0, 2000, 2200 };
const int auton_points [heightsAUTO::E_NUM_OF_HEIGHTS] = { 0, 750, 2000, 3000};

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
                g_timeout = pros::millis() + 1000;
                break;

                case(heights::E_MED):

                g_target = points[heights::E_MED];
                lift_state = heights::E_MED;
                g_clearTray = true;
                g_timeout = pros::millis() + 1000;
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

        while(pros::millis() < liftAUTO.getFailsafe()){ //&& pros::millis() < liftAUTO.getTimer()){

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



  void lift_task(void * param){
    okapi::ControllerButton l1(okapi::ControllerDigital::L1);
  	okapi::ControllerButton l2(okapi::ControllerDigital::L2);
    okapi::ControllerButton up(okapi::ControllerDigital::up);
    int iterator = 0;

    while(true){



      while(pros::competition::is_autonomous()){

        float final_power = liftAUTO.calculate(g_target, lift_mtr.get_position());

        if(pros::millis() < liftAUTO.getFailsafe()){ //&& pros::millis() < liftAUTO.getTimer()){
        lift_set(final_power);
       }
       else{
        lift_mtr.move_absolute(0, 180);
        lift_mtr.move_velocity(0); // hold and break
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

               if(l1.changedToPressed()){ //&& iterator < 1){
                ++iterator;
                pros::delay(500);
            //    if(tilter_mtr.get_position() > 1000 && tilter_mtr.get_position() < 2750){
                //  tilter_mtr.move_absolute(2000, 100);
            //      pros::delay(1000);
            //    }
                lift::setTarget(heights::E_MED, 3000);
                final_power = liftDRIVER.calculate(g_target, lift_mtr.get_position());

                if(pros::millis() < liftDRIVER.getFailsafe() && tilter::g_liftIsReady && g_readyToLift){
                lift_set(final_power);
              }

                else {
                //  lift_mtr.set_brake_mode(MOTOR_BRAKE_HOLD);
                  lift_mtr.move_velocity(0);
                }

          }



            if(l2.changedToPressed()){// && iterator > 0){
            iterator = 0;
            lift_mtr.move_absolute(0, 200);
            pros::delay(1000);
          //  tilter_mtr.move_absolute(0, 100);
            pros::delay(500);
            }

            if(up.changedToPressed()){

                              if(tilter::g_liftIsReady){//tilter_mtr.get_position() > 1000 && tilter_mtr.get_position() < 2750){
                              //  tilter_mtr.move_absolute(2000, 100);
                                pros::delay(1000);
                              }
                            //  lift_mtr.move_absolute(1800, 200);
            }
/*
        if(l1.changedToPressed() && iterator == 0){
            iterator = 1;
          }


        	if(l2.changedToPressed() && iterator == 1){
            	iterator = 0;
        	}
*/


                /*  if(iterator > 0){//&& tilter::get_target() == tilter::State_Machine::E_OFF){
                  //  tilter::setTarget(tilter::State_Machine::E_LIFT);
                  tilter_mtr.move_absolute(2000, 100);
                  pros::delay(500);
                  lift::setTarget(iterator);
                  lift_mtr.move_absolute(g_target, 100);
                  //  tilter_task_ctor->notify();
                    }
*/

                /*      else if(iterator > 0 && lift_task_ctor->notify_take(true, TIMEOUT_MAX) && tilter::get_target() == tilter::State_Machine::E_LIFT){
                          lift_mtr.move_absolute(g_target, 50);
                          }
*/ /*
                          while(iterator < 1){ //&& tilter::get_target() != tilter::State_Machine::E_OFF){
                            lift::setTarget(iterator);
                            lift_mtr.move_absolute(g_target, 50);
                            pros::delay(500);
                            tilter_mtr.move_absolute(0, 75);
                        //    tilter_task_ctor->notify();
                          }
                          */


                           // else {
                          //    lift_mtr.move_absolute(g_target, 50);
                            //  }


                  /*      else if(lift_task_ctor->notify_take(true, TIMEOUT_MAX) && tilter::get_target() == tilter::State_Machine::E_OFF){
                        lift_task_ctor->notify_clear();
                        tilter_task_ctor->notify_clear();
                        }
*/



    /*  if(lift_task_ctor->notify_take(true, TIMEOUT_MAX) && tilter::get_target() == tilter::State_Machine::E_LIFT){
      lift_mtr.move_absolute(g_target, 50);
    }
      */

       pros::delay(10);
      }
/*     else{
      tilt.set_state(Tilter::State_Machine::E_LIFT); }
*/
        pros::delay(10);
        }

// while(pros::competition::is_autonomous() && auton_flag){}

      }
//  }










}
