#include "main.h"
#include "PID.hpp"
#include "utilities.hpp"


void PID::reinit(double kp, double kd, double ki, double threshold, int max_vel, uint32_t settle, uint32_t max_time){

m_kP = kp;
m_kD = kd;
m_kI = ki;

m_maxVel = max_vel;
m_settle = settle;
m_max_time = max_time;
m_threshold = threshold;

}


double PID::calculateErr(double ierror){//double target, double pv){
  m_error = ierror;

  //calculate delta time
//  double dT = pros::millis() - m_lastTime;
  //abort if dt is too small
//  if(dT < m_minDt) return m_final_power;

  //calculate proportional
  m_proportional = m_error * m_kP;

  //calculate derivative
  m_derivative = m_kD * ((m_error - m_lastError)); /// dT);

  //calculate integral
  if(std::abs(m_error) < m_integral_active_zone && std::abs(m_error) > m_threshold){
  m_integral += m_kI * m_error;
  }
  else if(!m_integral_active_zone){
   m_integral = 0.0;
  }
  else { m_integral = 0.0; }

  if(std::abs(m_integral) > m_integral_active_zone){
      m_integral = m_integral_active_zone * sgn_(m_integral);
  }

  //calculate output
  m_final_power = (m_proportional) + (m_derivative) + (m_integral);
  //limit output
  if(std::abs(m_final_power) > m_maxVel) m_final_power = m_maxVel * sgn_(m_maxVel);

  //save values
  m_lastTime = pros::millis();
  m_lastError = m_error;

  return m_final_power;
}


double PID::calculateFailsafe(std::uint32_t i_max_time){
m_max_time = i_max_time;
m_failsafe = pros::millis() + i_max_time;
return m_failsafe;
}


double PID::calculateTimer(std::uint32_t isettle, double ierror){
m_settle = isettle;

if(ierror > m_threshold){
m_invoke_timer = true;
}

else if(!m_invoke_timer){
m_timer = pros::millis() + m_settle;
}

return m_timer;
}


double PID::calculate(double target, double pv){
return(calculateErr(target - pv));
}

double PID::getError(){
return m_error;
}

double PID::getTimer(){
return m_timer;
}

double PID::getFailsafe(){
return m_failsafe;
}

double PID::getSettle(){
return m_settle;
}

void PID::setThreshold(double ithreshold){
m_threshold = ithreshold;
}

//void PID::setTimeout(uint32_t itimeout){
//m_max_time = itimeout;
//}

void PID::setSettle(uint32_t isettle){
m_settle = isettle;
}

void PID::setInvoke(bool iinvoke){
m_invoke_timer = iinvoke;
}

void PID::setIntegralActiveZone(float izone){
m_integral_active_zone = izone;
}

void PID::setIntegralLimit(float ilimit){
m_integral_limit = ilimit;
}

void PID::setMaxVel(float ivel){
m_maxVel = ivel;
}


void PID::logTimer(){
std::cout << "timer" << m_timer << std::endl << std::endl;
std::cout << "failsafe" << m_failsafe << std::endl << std::endl;
std::cout << "settle" << m_settle << std::endl << std::endl;
}


void PID::reset(){
m_error = 0.0;
m_proportional = 0.0;
m_derivative = 0.0;
m_integral = 0.0;
m_final_power = 0.0;

m_lastError = 0.0;
m_lastTime = 0.0;

m_timer = 0;
}
