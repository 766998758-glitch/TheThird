#include "PID.h"
#include "ADC.h"
#include "Motor.h"
#include <math.h>

// 用于应对多重弯道
static float error_history[5] = {0};
static uint8_t history_index = 0;
static uint8_t continuous_curve = 0;
static uint8_t curve_side = 0; // 0无 1连续左弯 2连续右弯
static uint8_t curve_memory = 0; // 弯道记忆，延长弯道状态
static float weighted_error_sum = 0; // 加权误差和，对“近期误差”更加敏感

// 直线加速
static uint8_t straight_line_count = 0; // 直线状态计数器
static uint8_t is_straight_line = 0;    // 直线状态标志


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
	
	 /****直线加速****/
	  // 获取传感器状态
	  uint8_t sensor_states = Binary_GetSensorStates(sensor_values);
    
    // 检测是否处于直线状态（0110）
    if(sensor_states == 0x06) {
        straight_line_count++;
        if(straight_line_count > 4) { // 四次一周期
            is_straight_line = 1;
        }
    } else {
        straight_line_count = 0;
        is_straight_line = 0;
    }
	
	 // 动态基速调整，根据误差大小调整基本速度（非常好的替换版本，如果用传感器传值，是要写一堆if的）
    uint16_t dynamic_base_speed = base_speed;
    float abs_error = fabs(error);
    
		if(is_straight_line && !continuous_curve) {
        // 直线且不在弯道时加速
        dynamic_base_speed = base_speed * 2.0f; // 100%加速
        if(dynamic_base_speed > 750) dynamic_base_speed = 750; // 限制一下
    } else {
        // 非直线或者连续弯道时使用原逻辑
        if(abs_error < 1.0f) {
            dynamic_base_speed = base_speed + 50;
        } else if(abs_error < 2.5f) {
            dynamic_base_speed = base_speed;
        } else {
            dynamic_base_speed = base_speed * 0.8f;
            if(dynamic_base_speed < 200) dynamic_base_speed = 200;
        }
    }
    /****直线加速****/
	
	
		/******连续弯道检测内容******/
	  // 更新历史误差用于弯道检测  这个取余是干嘛的？
		error_history[history_index] = error;
    history_index = (history_index + 1) % 5;
		
		weighted_error_sum = 0;
    float weight_sum = 0;
    for(int i = 0; i < 5; i++) {
        float weight = 1.0f / (5 - i); // 近期误差权重更高
        weighted_error_sum += error_history[i] * weight;
        weight_sum += weight;
    }
    float weighted_error_avg = weighted_error_sum / weight_sum;
    
		// 连续弯道检测
		uint8_t new_continuous_curve = 0;
    uint8_t new_curve_side = 0;
    
    if(fabs(weighted_error_avg) > 1.3f) { // 降低阈值，更敏感
        new_continuous_curve = 1;
        new_curve_side = (weighted_error_avg > 0) ? 1 : 2;
    }
    
    // 连续弯道记忆：如果刚刚是连续弯道，那就保持这个状态更长
    if(continuous_curve && !new_continuous_curve) {
        curve_memory++;
        if(curve_memory < 3) { // 保持三个周期
            new_continuous_curve = 1;
            new_curve_side = curve_side;
        } else {
            curve_memory = 0;
        }
    } else {
        curve_memory = 0;
    }
    
    continuous_curve = new_continuous_curve;
    curve_side = new_curve_side;
    
    // 连续弯道时加强控制响应
    if(continuous_curve) {
        // 更强的误差信息放大
        error *= 1.7f;  
        
        // 根据历史趋势，预设更强的转向偏差
        if(curve_side == 1) {
            error += 1.2f;   
        } else if(curve_side == 2) {
            error -= 1.2f;   
        }
        
        pid->Ki *= 1.2f;
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
        pid_limit = 400.0f; // 连续弯道时提高限幅
    }
    if(fabs(error) > 3.0f) {
        pid_limit = 440.0f; // 大误差时提高限幅
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