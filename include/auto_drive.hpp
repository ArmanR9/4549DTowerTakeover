#ifndef AUTO_DRIVE_HPP
#define AUTO_DRIVE_HPP
#include "main.h"
#include "odometry.hpp"

void turn2ang(float angle, int max_velocity, _TurnDir direction = _TurnDir::CH , uint32_t settle = 300, uint32_t max_time = 5000, float ikI = 1.589);

void driveToPosition(float y, float x, float ys, float xs, float maxErrX, float maxVel, std::uint32_t ifailsafe, bool enableCorrect, bool forward, bool harshStop);
void driveToDistance(float d, float a, float ys, float xs, float maxErrX, float maxVel, std::uint32_t ifailsafe, bool enableCorrect, bool forward, bool harshStop);

void position_sweep(double y, double x, double ys, double xs, bool forward);

void motion_profile(float setpoint, float velocity);

void drive_lineup(int voltage, uint32_t give_time);
void turning_lineup(int voltage, uint32_t give_time);

void drive_outtake(int drive, int intake, uint32_t give_time);
void outtake(int voltage, uint32_t give_time);

void lineup_right(int voltage, uint32_t give_time);
void lineup_left(int voltage, uint32_t give_time);

void while_drive(int voltage, uint32_t give_time);

void drive_relative(double position, int voltage);

#endif
