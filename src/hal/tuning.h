#ifndef __TUNING_H__
#define __TUNING_H__

#include<SimpleFOC.h>

/** motor.cpp */
extern PIDController pid_vel;
extern PIDController pid_stb;
extern PIDController pid_vel_tmp;
extern PIDController pid_steering;
extern float g_zero_angle;
extern BLDCMotor motor_0;
extern BLDCMotor motor_1;

/** bot.cpp */
extern PIDController pid_bot_s;
extern PIDController pid_bot_m;
#endif