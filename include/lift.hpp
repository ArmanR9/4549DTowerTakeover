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


extern bool g_auton_flag;
extern bool g_opc_flag;


extern const int points[];

void setTarget(int setTarget);

heights::lift getTarget();


void lift_task(void* param);
}


#endif
