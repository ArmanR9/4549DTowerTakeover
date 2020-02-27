#ifndef LIFT_HPP
#define LIFT_HPP
#include "main.h"

namespace lift {

namespace heights
{
    enum lift
    {
        E_OFF, // 0
        E_LOW, // 1
        E_MED, // 2
        E_NUM_OF_HEIGHTS // 3
    };
}

namespace heightsAUTO{

  enum auton
  {
  E_OFF, // 0
  E_CUBES, // 1
  E_LOW, // 2
  E_MED, // 3
  E_NUM_OF_HEIGHTS // 4
  };
}


extern bool g_auton_flag;
extern bool g_opc_flag;
extern bool g_readyToLift;
extern bool g_clearTray;
extern std::uint32_t g_timeout;


extern const int points[];

void autonLift(int height, std::uint32_t timeoutLIFT, std::uint32_t timeoutTILT);
void setTarget(int setTarget, std::uint32_t ifailsafe = 10000);
void setTargetAuton(int setTarget, std::uint32_t failsafe);
void setTargetAutonAsync(int setTarget, std::uint32_t failsafe);


heights::lift getTarget();


void lift_task(void* param);
}

extern pros::Mutex lift_mutex;
extern pros::Mutex off_mutex;

#endif
