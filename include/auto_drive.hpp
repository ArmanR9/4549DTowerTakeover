#ifndef AUTO_DRIVE_HPP
#define AUTO_DRIVE_HPP
#include "main.h"


#define RIGHT_DIRECTION 1
#define LEFT_DIRECTION 2 // macros for arc turns

void motion_profile(float setpoint, float velocity);

void drive_point(double targetX, double targetY, double startX, double startY, double startA, float velocity, uint32_t settle, uint32_t max_time);
void drive_pid(int setpoint, int max_speed, unsigned int timeout);
void drive_encoder(int setpoint, int max_speed, float currError);
void turning_pid(int degrees, int max_speed);
void straight_line_drive(int setpoint, int max_speed, unsigned int timeout, float angle);

#endif
