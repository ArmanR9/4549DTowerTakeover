#ifndef PID_HPP
#define PID_HPP

class PID{
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

  void reinit(double kp, double kd, double ki, int maxvel, uint32_t settle, uint32_t max_time);
  double calculateErr(double target, double pv);
  double calculate(double x, double y);
  double getError();
  void reset();



  PID(double kP, double kD, double kI, double maxVel, uint32_t settle, uint32_t max_time) :
   m_kP(kP),
   m_kD(kD),
   m_kI(kI),
   m_maxVel(maxVel),
   m_settle(settle),
   m_max_time(max_time)
   {}
};



#endif