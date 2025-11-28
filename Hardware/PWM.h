#ifndef __PWM_H
#define __PWM_H
#include "stm32f10x.h"                  // Device header

// PWM通道定义
typedef enum {
    PWM_CHANNEL_LEFT = 1,   // PA8
    PWM_CHANNEL_RIGHT = 4   // PA11
} PWM_Channel;


void PWM_Init(void);
void PWM_SetDutyCycle(PWM_Channel channel, uint16_t duty);

#endif
