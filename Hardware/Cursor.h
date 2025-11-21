#ifndef __CURSOR_H
#define __CURSOR_H
#include "stm32f10x.h"                  // Device header


struct PID {
    float kp;
    float ki;
    float kd;
};

extern uint8_t current_line;
extern uint8_t Key;
extern uint16_t Flag;
extern struct PID pid;


void Cursor_Init(void);
void Cursor_Tick(void);

#endif

