#include "intake.hpp"
#include "main.h"
#include "motors.hpp"
#include "sensors.hpp"

//TilterPID tilt(0,0,0,0,0,0);
namespace intake{

  int g_target;
  uint32_t g_set_time;
  States current_state;


  void setTarget(int voltage, uint32_t give_time){
      intake_set(voltage);
      pros::delay(give_time);
    }


  void set_targetAsync(States setTarget, uint32_t give_time){
      g_set_time = pros::millis() + give_time;
      current_state = setTarget;


      switch(setTarget){

            case(States::E_INTAKE):

                g_target = 127;
                break;

                case(States::E_OUTTAKE):

                g_target = -127;
                break;

                case(States::E_DEPLOY):

                g_target = -50;
                break;

                case(States::E_OFF):

                g_target = 0;
                break;
              }

      }



      States getTarget(){
        return current_state;
      }



  void intake_task(void * param){
    uint32_t timer = 0;

    while(true){

      while(pros::competition::is_autonomous()){
        if(g_set_time > pros::millis()){
        intake_set(g_target);
          }
      else intake_set(0);

        pros::delay(10);
      }
/*      else{
      tilt.set_state(Tilter::State_Machine::E_LIFT); }
*/
        pros::delay(10);
        }

// while(pros::competition::is_autonomous() && auton_flag){}

      }
//  }










}
