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
extern bool g_outtake;

void set_targetAsync(States voltage, uint32_t give_time);
void setTarget(int voltage, uint32_t give_time);
void deploy(std::uint32_t timeout);
void deployOff();
void light_senAsync(std::uint32_t timeout, int hi = -110, int lo = -80);
void light_senOff();
void light_sen(std::uint32_t timeout);

States getTarget();


void intake_task(void* param);
}



#endif
