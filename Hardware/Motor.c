#include "Motor.h"
#include "PWM.h"

//电机初始化
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    //方向控制引脚 PA0, PA1, PA2, PA3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //初始化是停止的
    GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
    
    //在这初始化PWM
    PWM_Init();
}

//设置电机速度
void Motor_SetSpeed(Motor_Type motor, uint16_t speed)
{
    if(speed > 999) speed = 999;
    
    switch(motor) {
        case MOTOR_LEFT:
            PWM_SetDutyCycle(PWM_CHANNEL_LEFT, speed);
            break;
        case MOTOR_RIGHT:
            PWM_SetDutyCycle(PWM_CHANNEL_RIGHT, speed);
            break;
    }
}

//设置电机方向
void Motor_SetDirection(Motor_Type motor, Direction_Type direction)
{
    switch(motor) {
        case MOTOR_LEFT:
            if(direction == DIR_FORWARD) {
                GPIO_SetBits(GPIOA, GPIO_Pin_0);
                GPIO_ResetBits(GPIOA, GPIO_Pin_1);
            } else if(direction == DIR_BACKWARD) {
                GPIO_ResetBits(GPIOA, GPIO_Pin_0);
                GPIO_SetBits(GPIOA, GPIO_Pin_1);
            } else {
                GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);
            }
            break;
            
        case MOTOR_RIGHT:
            if(direction == DIR_FORWARD) {
                GPIO_SetBits(GPIOA, GPIO_Pin_2);
                GPIO_ResetBits(GPIOA, GPIO_Pin_3);
            } else if(direction == DIR_BACKWARD) {
                GPIO_ResetBits(GPIOA, GPIO_Pin_2);
                GPIO_SetBits(GPIOA, GPIO_Pin_3);
            } else {
                GPIO_ResetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
            }
            break;
    }
}

//循迹结束
void Car_Stop(void)
{
    Motor_SetDirection(MOTOR_LEFT, DIR_STOP);
    Motor_SetDirection(MOTOR_RIGHT, DIR_STOP);
    Motor_SetSpeed(MOTOR_LEFT, 0);
    Motor_SetSpeed(MOTOR_RIGHT, 0);
}

//小车运动控制
void Car_Move(uint8_t direction, uint16_t speed)
{
    switch(direction) {
        case 1: // 前进
            Motor_SetDirection(MOTOR_LEFT, DIR_FORWARD);
            Motor_SetDirection(MOTOR_RIGHT, DIR_FORWARD);
            Motor_SetSpeed(MOTOR_LEFT, speed);
            Motor_SetSpeed(MOTOR_RIGHT, speed);
            break;
        case 2: // 后退
            Motor_SetDirection(MOTOR_LEFT, DIR_BACKWARD);
            Motor_SetDirection(MOTOR_RIGHT, DIR_BACKWARD);
            Motor_SetSpeed(MOTOR_LEFT, speed);
            Motor_SetSpeed(MOTOR_RIGHT, speed);
            break;
        case 3: // 左转
            Motor_SetDirection(MOTOR_LEFT, DIR_BACKWARD);
            Motor_SetDirection(MOTOR_RIGHT, DIR_FORWARD);
            Motor_SetSpeed(MOTOR_LEFT, speed/2);
            Motor_SetSpeed(MOTOR_RIGHT, speed/2);
            break;
        case 4: // 右转
            Motor_SetDirection(MOTOR_LEFT, DIR_FORWARD);
            Motor_SetDirection(MOTOR_RIGHT, DIR_BACKWARD);
            Motor_SetSpeed(MOTOR_LEFT, speed/2);
            Motor_SetSpeed(MOTOR_RIGHT, speed/2);
            break;
        default:
            Car_Stop();
            break;
    }
}
