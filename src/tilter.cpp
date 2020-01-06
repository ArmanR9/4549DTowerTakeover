#include "tilter.hpp"
#include "main.h"
#include "motors.hpp"
#include "sensors.hpp"

namespace tilter{

  bool g_auton_flag;
  bool g_opc_flag;

  tilter::State_Machine state;
  int g_target;





   tilter::State_Machine get_state(){
   return state;
   }

  void tilter_task(void * param){

    while(true){

      while(!pros::competition::is_autonomous() && g_opc_flag){

        RB_mtr.move(g_target);
        pros::delay(10);
      }

    pros::delay(10);
    }
  }


}


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
