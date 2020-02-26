#ifndef TILTER_HPP
#define TILTER_HPP
#include "main.h"
#include "motors.hpp"



namespace tilter {
extern bool g_auton_flag;
extern bool g_opc_flag;
extern bool g_clearTray;
extern bool g_readyToStack;
extern bool g_liftIsReady;
extern std::uint32_t g_timeout;
//extern bool isDone;

enum class State_Machine{
E_OFF, // 0
E_STACK, // 1
E_LIFT // 2
};


void setTarget(State_Machine setTarget, std::uint32_t ifailsafe = 10000);

bool isSettled();
void waitUntilSettled(uint32_t timeout);
State_Machine get_target();

void tilter_task(void * param);


}


#endif
