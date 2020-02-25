#ifndef INTAKE_HPP
#define INTAKE_HPP

#include "main.h"

namespace intake {


    enum class States
    {
        E_INTAKE, // 0
        E_OUTTAKE, // 1
        E_DEPLOY, // 2
        E_OFF // 3
    };



extern bool g_auton_flag;
extern bool g_opc_flag;

void set_targetAsync(States voltage, uint32_t give_time);
void setTarget(int voltage, uint32_t give_time);
void deploy(uint32_t give_time);

States getTarget();


void intake_task(void* param);
}



#endif
