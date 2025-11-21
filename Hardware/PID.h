#ifndef __PID_H
#define __PID_H
#include "stm32f10x.h"                  // Device header


// PID参数结构体
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float previous_error;
    float integral_limit;
} PID_Controller;


void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float integral_limit);
float PID_Calculate(PID_Controller *pid, float error);
float LineFollower_CalculateError(uint16_t *sensor_values);
void LineFollower_Update(PID_Controller *pid, uint16_t base_speed);

#endif
