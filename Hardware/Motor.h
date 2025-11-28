#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f10x.h"                  // Device header


//电机枚举
typedef enum {
    MOTOR_LEFT = 1,
    MOTOR_RIGHT = 2
} Motor_Type;

//方向枚举
typedef enum {
    DIR_STOP = 0,
    DIR_FORWARD = 1,
    DIR_BACKWARD = 2
} Direction_Type;


void Motor_Init(void);
void Motor_SetSpeed(Motor_Type motor, uint16_t speed);
void Motor_SetDirection(Motor_Type motor, Direction_Type direction);
void Car_Stop(void);
void Car_Move(uint8_t direction, uint16_t speed);

#endif
