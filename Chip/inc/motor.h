#ifndef __MOTOR_H__
#define __MOTOR_H__
#endif

#include "common.h"
#include "include.h"

extern volatile uint32 Moduty;

#define MOTOR_FTM   FTM0
#define MOTOR1_PWM  FTM_CH3
#define MOTOR1_IO   PTA6

void Motor (void);
void MotorInit();
void MotorDuty();