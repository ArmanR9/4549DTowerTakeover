#include "main.h"
#include "PID.hpp"


void PID::reinit(double kp, double ki, double kd, int max_vel, uint32_t settle, uint32_t max_time){

m_kP = kp;
m_kD = kd;
m_kI = ki;

m_maxVel = max_vel;
m_settle = settle;
m_max_time = max_time;

}


double PID::getError(){
return m_error;
}


double PID::calculateErr(double target, double pv){

m_error = target - pv;


return m_error;
}