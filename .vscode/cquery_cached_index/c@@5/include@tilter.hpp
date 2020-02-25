#ifndef TILTER_HPP
#define TILTER_HPP
#include "main.h"
#include "motors.hpp"


/*
class Tilter{
public:

  enum class State_Machine : int{
    E_OFF, // 0
    E_STACK, // 1
    E_LIFT, // 2
  };

  enum angles : int{
    E_OFF_A, // 0
    E_STACK_A, // 1
    E_LIFT_A, // 2
    E_NUM_OF_ANGLES// 3
  };

private:


  State_Machine m_curState;
  State_Machine m_targetState;

  double m_target;
  double m_curAngle;
  const int m_angles[E_NUM_OF_ANGLES] { 0, 50, 100};


public:

  bool auton_flag;
  bool opc_flag;

    //Methods
    State_Machine get_state(){return m_curState;}
    double get_angle(){ return m_curAngle;}

    // Defined in source file
    void set_state(State_Machine s);

    void computeTarget(State_Machine& target_state);


Tilter() : m_curAngle(tilter_mtr.get_position()){}
};




class TilterPID : public Tilter{

private:
  double m_kP;
  double m_kD;
  double m_kI;

  unsigned int dT;
  unsigned int m_minDt;

  bool m_invoke_timer;
  uint32_t m_failsafe;
  uint32_t m_max_time;
  uint32_t m_settle;
  float  m_threshold;

  double m_error;
  double m_lastError;
  double m_lastTime;

  bool m_integral_active_zone;
  float integral_limit;

  double m_proportional;
  double m_integral;
  double m_derivative;
  double m_final_power ;

  double m_maxVel;


public:

  double calculateErr(double);
  double calculate(double, double);
  double getError();
  void reset();


  TilterPID(double kP, double kD, double kI, double maxVel, uint32_t settle, uint32_t max_time) :
   m_kP(kP),
   m_kD(kD),
   m_kI(kI),
   m_maxVel(maxVel),
   m_settle(settle),
   m_max_time(max_time)
   {}


};
*/




namespace tilter {
extern bool g_auton_flag;
extern bool g_opc_flag;

enum class State_Machine{
E_OFF, // 0
E_STACK, // 1
E_LIFT // 2
};


void setTarget(State_Machine setTarget);
State_Machine get_target();

void tilter_task(void * param);


}


#endif
