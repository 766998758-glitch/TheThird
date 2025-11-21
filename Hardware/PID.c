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
    
    //积分限幅
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
    float weighted_sum = 0;
    float sum = 0;
    
    for(int i = 0; i < 4; i++) {
        weighted_sum += sensor_values[i] * (i - 1.5);  // 权重:-1.5, -0.5, 0.5, 1.5
        sum += sensor_values[i];
    }
    
    if(sum == 0) return 0;  //如果四个传感器都传白的
    
    return weighted_sum / sum;
}

//循迹更新，我感觉这辈子都没设计过这么麻烦的东西
void LineFollower_Update(PID_Controller *pid, uint16_t base_speed)
{
    uint16_t sensor_values[4];
    
    //读取传感器
    ADC_ReadAllSensors(sensor_values);
    
    //计算误差
    float error = LineFollower_CalculateError(sensor_values);
    
    // PID计算
    float pid_output = PID_Calculate(pid, error);
    
    //计算左右轮速度
    uint16_t left_speed = base_speed - pid_output;
    uint16_t right_speed = base_speed + pid_output;
    
    //速度限幅
    if(left_speed > 999) left_speed = 999;
    if(left_speed < 0) left_speed = 0;
    if(right_speed > 999) right_speed = 999;
    if(right_speed < 0) right_speed = 0;
    
    // 设置电机前进方向
    Motor_SetDirection(MOTOR_LEFT, DIR_FORWARD);
    Motor_SetDirection(MOTOR_RIGHT, DIR_FORWARD);
    Motor_SetSpeed(MOTOR_LEFT, left_speed);
    Motor_SetSpeed(MOTOR_RIGHT, right_speed);
}
