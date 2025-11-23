#include "PID.h"
#include "ADC.h"
#include "Motor.h"

// PID初始化
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float integral_limit)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0;
    pid->previous_error = 0;
    pid->integral_limit = integral_limit;
}

// PID计算
float PID_Calculate(PID_Controller *pid, float error)
{
    pid->integral += error;
    
    // 积分限幅
    if(pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    
    float derivative = error - pid->previous_error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    pid->previous_error = error;
    return output;
}

//计算循迹误差
float LineFollower_CalculateError(uint16_t *sensor_values)
{
    // X41检测白区 X32检测黑线

    // 权重计算
    float weighted_sum = 0;
    float sum = 0;
    
    // 给每个传感器分配权重，检测黑线的权重更高
    // 目前权重（之后会调整）:X1(-2.0), X2(-0.5), X3(0.5), X4(2.0)
    weighted_sum = sensor_values[0] * (-2.0f) +  
                   sensor_values[1] * (-0.5f) +  
                   sensor_values[2] * (0.5f) +   
                   sensor_values[3] * (2.0f);  
    
    sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3];
    
    if(sum == 0) return 0;  // 防止除0
    
    return weighted_sum / sum;
}

// 循迹更新 - 修复比较数无符号问题
void LineFollower_Update(PID_Controller *pid, uint16_t base_speed)
{
    uint16_t sensor_values[4];
    
    // 读取传感器
    ADC_ReadAllSensors(sensor_values);
    
    // 计算误差
    float error = LineFollower_CalculateError(sensor_values);
    
    // PID计算
    float pid_output = PID_Calculate(pid, error);
    
    // 计算左右轮速度
    int32_t left_speed_temp = (int32_t)base_speed - (int32_t)pid_output;
    int32_t right_speed_temp = (int32_t)base_speed + (int32_t)pid_output;
    
    // 速度限幅
    if(left_speed_temp > 999) left_speed_temp = 999;
    if(left_speed_temp < 0) left_speed_temp = 0;
    if(right_speed_temp > 999) right_speed_temp = 999;
    if(right_speed_temp < 0) right_speed_temp = 0;
    
    // 转换成无符号数
    uint16_t left_speed = (uint16_t)left_speed_temp;
    uint16_t right_speed = (uint16_t)right_speed_temp;
    
    // 设置电机前进方向
    Motor_SetDirection(MOTOR_LEFT, DIR_FORWARD);
    Motor_SetDirection(MOTOR_RIGHT, DIR_FORWARD);
    Motor_SetSpeed(MOTOR_LEFT, left_speed);
    Motor_SetSpeed(MOTOR_RIGHT, right_speed);
}