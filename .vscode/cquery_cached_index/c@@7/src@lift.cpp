#include "lift.hpp"
#include "main.h"
#include "motors.hpp"
#include "sensors.hpp"
#include "tilter.hpp"
#include "tasks.hpp"

pros::Mutex lift_mutex;
pros::Mutex off_mutex;
//TilterPID tilt(0,0,0,0,0,0);
namespace lift{

  bool g_auton_flag;
  bool g_opc_flag;

  int g_target;

  heights::lift lift_state = heights::E_OFF;


const int points [heights::E_NUM_OF_HEIGHTS] = { 0, 2000, 2000 };


  void setTarget(int setTarget){

      switch(setTarget){

            case(heights::E_OFF):

                g_target = points[heights::E_OFF];
                lift_state = heights::E_OFF;
                break;

                case(heights::E_LOW):

                g_target = points[heights::E_LOW];
                lift_state = heights::E_LOW;
                break;

                case(heights::E_MED):

                g_target = points[heights::E_MED];
                lift_state = heights::E_MED;
                break;
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

     while(!pros::competition::is_autonomous()){

        if(lift_mtr.get_position() > 500 && lift_mtr.get_position() < 1500){
            lift_state = heights::E_LOW;
          }
          else if(tilter_mtr.get_position() > 1501){
              lift_state = heights::E_MED;
            }
            else { lift_state = heights::E_OFF; }

               if(l1.changedToPressed()){ //&& iterator < 1){
                ++iterator;

                if(tilter_mtr.get_position() > 1000 && tilter_mtr.get_position() < 2750){
                //  tilter_mtr.move_absolute(2000, 100);
                  pros::delay(1000);
                }
                lift_mtr.move_absolute(2300, 200);
          }



            if(l2.changedToPressed()){// && iterator > 0){
            iterator = 0;
            lift_mtr.move_absolute(0, 200);
            pros::delay(1000);
          //  tilter_mtr.move_absolute(0, 100);
            pros::delay(500);
            }

            if(up.changedToPressed()){

                              if(tilter_mtr.get_position() > 1000 && tilter_mtr.get_position() < 2750){
                              //  tilter_mtr.move_absolute(2000, 100);
                                pros::delay(1000);
                              }
                              lift_mtr.move_absolute(1800, 200);
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
