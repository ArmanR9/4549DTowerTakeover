#ifndef AUTO_DRIVE_HPP
#define AUTO_DRIVE_HPP
#include "main.h"
#include "odometry.hpp"


#define RIGHT_DIRECTION 1
#define LEFT_DIRECTION 2 // macros for arc turns

void motion_profile(float setpoint, float velocity);

void drive_point(double targetX, double targetY, double startX, double startY, double startA, float velocity, uint32_t settle, uint32_t max_time);
void drive_pid(int setpoint, int max_speed, unsigned int timeout);
void drive_encoder(int setpoint, int max_speed, float currError);
void turning_pid(int degrees, int max_speed);

void straight_line_drive(float setpoint, float angle, float kP_correction, float maxErrA, int max_velocity, uint32_t settle, uint32_t max_time);
void turn2ang(float angle, int max_velocity, _TurnDir direction = _TurnDir::CH , uint32_t settle = 300, uint32_t max_time = 5000);

void driveToPosition(float y, float x, float ys, float xs, float maxErrX, float maxVel, bool enableCorrect, bool forward, bool harshStop);


void drive_lineup(int voltage, uint32_t give_time);
void drive_outtake(int drive, int intake, uint32_t give_time);
void outtake(int voltage, uint32_t give_time);
void lineup_right(int voltage, uint32_t give_time);
void lineup_left(int voltage, uint32_t give_time);
void while_drive(int voltage, uint32_t give_time);

#endif
