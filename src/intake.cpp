#include "intake.hpp"
#include "main.h"
#include "motors.hpp"
#include "sensors.hpp"
#include "utilities.hpp"

//TilterPID tilt(0,0,0,0,0,0);
namespace intake{

  int g_target;
  uint32_t g_set_time;
  States current_state;
  bool g_readyToStack = false;
  bool g_outtake = false;
  std::uint32_t g_timeout;

  int g_hi = -110;
  int g_lo = -80;

 
void light_sen(std::uint32_t timeout){

    if(light_sensor.get_value() > light_sensor_threshold3){
            int ispeed = pow(light_sensor.get_value()*0.01, 1.25);
            ispeed = std::clamp(ispeed, -120, -100);
            intake_set(ispeed);
            pros::delay(timeout);
          }
      else intake_set(0);
   }

  void light_senAsync(std::uint32_t timeout, int hi, int lo){
  g_outtake = true;
  g_timeout = pros::millis() + timeout;
  g_hi = hi;
  g_lo = lo;
  }

  void light_senOff(){
  g_outtake = false;      
  }

  void deploy(std::uint32_t timeout){
  g_target = -80;
  g_readyToStack = true;
  g_timeout = pros::millis() + timeout;
  }

  void deployOff(){
  g_readyToStack = false;
  }

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

                g_target = -97;
                break;

                case(States::E_OFF):

                g_target = 0;
                break;
              }

      }

//      void deploy(uint32_t give_time){
    //  intake_set(-127);
    //  pros::delay(give_time);
    //  }


      States getTarget(){
        return current_state;
      }



  void intake_task(void * param){
    uint32_t timer = 0;

    while(true){

      while(pros::competition::is_autonomous()){
        if(g_set_time > pros::millis() && !g_outtake){
        intake_set(g_target);
          }
          else if(light_sensor.get_value() > 1850 && g_outtake && pros::millis() < g_timeout){
            int ispeed = pow(light_sensor.get_value()*0.01, 1.25);
            ispeed = std::clamp(ispeed, g_hi, g_lo);
            intake_set(ispeed);
          }
      else{
       intake_set(0);
      }
      //  if(light_sensor.get_value() > light_sensor_threshold && g_readyToStack && pros::millis() < g_timeout){
      //  intake_set(g_target);
      //  }
      //  else { intake_set(0);
    //    g_readyToStack = false;

        std::cout << "intake sen" << light_sensor.get_value() << '\n';
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
