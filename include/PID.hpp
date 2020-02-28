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
  uint32_t m_timer;
  float m_threshold;

  double m_error;
  double m_lastError;
  double m_lastTime;

  bool m_integral_active_zone;
  float m_integral_limit;

  double m_proportional;
  double m_integral;
  double m_derivative;
  double m_final_power ;

  double m_maxVel;

  public:

  // Methods

  void reinit(double kp, double kd, double ki, double threshold, int maxvel, uint32_t settle, uint32_t max_time);



  double calculateErr(double ierror);
  double calculateFailsafe(std::uint32_t i_max_time);
  double calculateTimer(std::uint32_t isettle, double ierror);

  double calculate(double target, double pv);




  void setThreshold(double ithreshold);
  //void setTimeout(uint32_t itimeout);
  void setSettle(uint32_t settle);
  void setInvoke(bool iinvoke);

  void setIntegralActiveZone(float izone);
  void setIntegralLimit(float ilimit);

  void setMaxVel(float ivel);


  double getError();
  double getTimer();
  double getFailsafe();
  double getFinalPower();
  double getSettle();
  double getMaxVel();

  void reset();

  void logTimer();



  PID(double kP, double kD, double kI, double threshold, double maxVel, uint32_t settle, uint32_t max_time) :
   m_kP(kP),
   m_kD(kD),
   m_kI(kI),
   m_threshold(threshold),
   m_maxVel(maxVel),
   m_settle(settle),
   m_max_time(max_time)
   {}
};



#endif
