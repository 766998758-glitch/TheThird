#include "PID.h"
#include "ADC.h"
#include "Motor.h"
#include <math.h>

// 用于应对多重弯道
static float error_history[3] = {0};
static uint8_t history_index = 0;
static uint8_t continuous_curve = 0;
static uint8_t curve_side = 0; // 0无 1连续左弯 2连续右弯

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
    // 使用二进制版本
    return BinaryLineFollower_CalculateError(sensor_values);
}

// 循迹更新 - 修复比较数无符号问题
void LineFollower_Update(PID_Controller *pid, uint16_t base_speed)
{
    uint16_t sensor_values[4];
		ADC_ReadAllSensors(sensor_values);
    
    // 读取传感器
    ADC_ReadAllSensors(sensor_values);
    
    // 计算误差
    float error = LineFollower_CalculateError(sensor_values);
	
	 // 动态基速调整，根据误差大小调整基本速度（非常好的替换版本，如果用传感器传值，是要写一堆if的）
    uint16_t dynamic_base_speed = base_speed;
    float abs_error = fabs(error);
    
    if(abs_error < 1.0f) {
        // 直行时保持较高速度
        dynamic_base_speed = base_speed + 50;
    } else if(abs_error < 2.5f) {
        // 小弯时保持原速
        dynamic_base_speed = base_speed;
    } else {
        // 大弯适当降低但别太慢
        dynamic_base_speed = base_speed * 0.8f;
        if(dynamic_base_speed < 200) dynamic_base_speed = 200; // 最低限幅
    }
	
	
		/******连续弯道检测内容******/
	  // 更新历史误差用于弯道检测  这个取余是干嘛的？
		error_history[history_index] = error;
    history_index = (history_index + 1) % 3;
    
		// 连续弯道检测
		float error_sum = error_history[0] + error_history[1] + error_history[2];
    float error_avg = error_sum / 3.0f;
	
		if(fabs(error_avg) > 1.5f) {
        continuous_curve = 1;
        curve_side = (error_avg > 0) ? 1 : 2;
    } else {
        continuous_curve = 0;
        curve_side = 0;
    }
	
		// 连续弯道时增强响应控制
    if(continuous_curve) {
        // 增强误差信号
        error *= 1.5f;
        // 略微提高ki，消除稳态误差
        pid->Ki *= 1.2f;
    }
		
		if(curve_side == 1) { // 连续左弯
        error += 0.8f;   // 预设左弯
    } else if(curve_side == 2) { // 连续右弯
        error -= 0.8f;   // 预设右弯
    }
		/******连续弯道检测内容******/
		
    // PID计算
    float pid_output = PID_Calculate(pid, error);
	
		// 恢复ki，如果被修改的话
		if(continuous_curve) {
        pid->Ki /= 1.2f;
    }
		
		// 动态限幅
    float pid_limit = 250.0f;
    if(continuous_curve) {
        pid_limit = 350.0f; // 连续弯道时提高限幅
    }
    if(fabs(error) > 3.0f) {
        pid_limit = 400.0f; // 大误差时提高限幅
    }
    
		if(pid_output > pid_limit) pid_output = pid_limit;
    if(pid_output < -pid_limit) pid_output = -pid_limit;
		
    // 计算左右轮速度
    int32_t left_speed_temp = (int32_t)dynamic_base_speed - (int32_t)pid_output;
    int32_t right_speed_temp = (int32_t)dynamic_base_speed + (int32_t)pid_output;
    
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