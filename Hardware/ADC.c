#include "ADC.h"


void ADC1_Module_Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    
    //时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    
    //ADC引脚配置 PA4, PA5, PA6, PA7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // ADC配置
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    //开！
    ADC_Cmd(ADC1, ENABLE);
    
    //校准
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

uint16_t ADC_ReadSingleSensor(uint8_t channel)
{
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_239Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    return ADC_GetConversionValue(ADC1);
}

void ADC_ReadAllSensors(uint16_t *sensor_values)
{
    sensor_values[SENSOR_X2] = ADC_ReadSingleSensor(ADC_Channel_4);  // PA4 -X2最左侧
    sensor_values[SENSOR_X1] = ADC_ReadSingleSensor(ADC_Channel_5);  // PA5 -X1中心靠左
    sensor_values[SENSOR_X3] = ADC_ReadSingleSensor(ADC_Channel_6);  // PA6 -X3中心靠右
    sensor_values[SENSOR_X4] = ADC_ReadSingleSensor(ADC_Channel_7);  // PA7 -X4最右侧
}


// 获取二进制传感器状态
uint8_t Binary_GetSensorStates(uint16_t *sensor_values)
{
    uint8_t states = 0;
    
    for(int i = 0; i < 4; i++) {
        if(sensor_values[i] < BLACK_THRESHOLD) {
            states |= (1 << i);  // 检测到黑线，对应状态1
        }
    }
    
    return states;
}

// 二进制式误差计算(主体)，我们用位掩码
float BinaryLineFollower_CalculateError(uint16_t *sensor_values)
{
    uint8_t sensor_states = Binary_GetSensorStates(sensor_values);
	
		static float previous_error = 0.0f; // 保存当前误差用于白区自救
		static uint8_t lost_count = 0;
		static uint8_t curve_direction = 0;  // 0直行，1左弯道，2右弯道
    
	 // 检测弯道趋势
    if(sensor_states == 0x04 || sensor_states == 0x0C || sensor_states == 0x08) {
        curve_direction = 1; // 左转趋势
    } else if(sensor_states == 0x02 || sensor_states == 0x03 || sensor_states == 0x01) {
        curve_direction = 2; // 右转趋势
    } else if(sensor_states == 0x06) {
        curve_direction = 0; // 直行
    }
	
    // 根据传感器状态计算误差
    switch(sensor_states) {
        case 0x06:   // 0110 x13在黑线，直行
						previous_error = 0.0f;
            return 0.0f;
            
        case 0x02: // 0010 仅x1检测黑线，略微右偏
            previous_error = -1.8f;
            return -1.8f;
            
        case 0x04: // 0100 仅x3检测黑线，略微左偏 
            previous_error = 1.8f;
            return 1.8f;
            
        case 0x03: // 0011 x21检测黑，明显右偏 
            previous_error = -2.8f;
            return -2.8f;
            
        case 0x0C: // 1100 x43检测黑，明显左偏 
            previous_error = 2.8f;
            return 2.8f;
            
        case 0x07: // 0111 大幅右偏
						previous_error = -3.5f;
            return -3.5f;
            
        case 0x0E: // 1110 大幅左偏
            previous_error = 3.5f;
            return 3.5f;
            
        case 0x01: // 0001 极右偏
            previous_error = -4.2f;
            return -2.5f;
            
        case 0x08: // 1000 极左偏 
            previous_error = 4.2f;
            return 2.5f;
            
        case 0x0F: // 1111 十字路口直行 
            lost_count = 0;
            // 根据来到十字路口时的车头朝向情况(因为不可能是直着来的)，进行临时转向，以此循迹
				if(curve_direction == 1)
				{
					return 1.5f;  // 右弯趋势，轻微左转
				}
				else if(curve_direction == 2)
				{
					return -1.5f; //同上，反之
				}
				else
				{
					return 0.0f;
				}
				
        case 0x00: // 0000 白区紧急自救 
            lost_count++;
				// 动态增强的自救策略
            float rescue_strength = 8.0f + (lost_count * 1.0f);
            if(rescue_strength > 12.0f) rescue_strength = 12.0f;
            
				// 根据目前弯道趋势，优化自救方向
				if(curve_direction == 1 || previous_error >0)
				{
					return rescue_strength;
				}
				else
				{
					return -rescue_strength;
				}
        default:
            return 0.0f;
    }
}

// 获取路线位置，返回-3~3的整数
int8_t Binary_GetLinePosition(uint16_t *sensor_values)
{
    uint8_t states = Binary_GetSensorStates(sensor_values);
    
    // 位置权重计算
    int8_t position = 0;
    
    if(states & (1 << SENSOR_X2)) position -= 2; // X2
    if(states & (1 << SENSOR_X1)) position -= 1; // X1
    if(states & (1 << SENSOR_X3)) position += 1; // X3
    if(states & (1 << SENSOR_X4)) position += 2; // X4
    
    return position;
}