#include "lift.hpp"
#include "main.h"
#include "motors.hpp"
#include "sensors.hpp"
#include "tilter.hpp"

//TilterPID tilt(0,0,0,0,0,0);
namespace lift{

  bool g_auton_flag;
  bool g_opc_flag;

  int g_target;

  heights::lift lift_state = heights::E_OFF;


const int points [heights::E_NUM_OF_HEIGHTS] = { 0, 50, 100 };


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

    while(true){

      while(!pros::competition::is_autonomous() && lift::g_opc_flag){
      //  if(tilt.get_state() == Tilter::State_Machine::E_LIFT){
        RB_mtr.move(g_target);
      }
/*      else{
      tilt.set_state(Tilter::State_Machine::E_LIFT); }
*/
        pros::delay(10);
        }

// while(pros::competition::is_autonomous() && auton_flag){}

      pros::delay(10);
      }
//  }










}
