#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"

#define IR_SENSOR_COUNT 4
#define BLACK_THRESHOLD 1500   // 黑线阈值（低于此值为黑线）
#define WHITE_THRESHOLD 2500   // 白区阈值（高于此值为白区）

//二进制传感器
typedef enum {
    SENSOR_WHITE = 0,
    SENSOR_BLACK = 1
} Sensor_State;

typedef enum {
    SENSOR_X2 = 0,  // 最左侧
    SENSOR_X1 = 1,  // 中心靠左
    SENSOR_X3 = 2,  // 中心靠右
    SENSOR_X4 = 3   // 最右侧
} Sensor_Position;

void ADC1_Module_Init(void);  
void ADC_ReadAllSensors(uint16_t *sensor_values);
uint16_t ADC_ReadSingleSensor(uint8_t channel);

//二进制循迹函数
uint8_t Binary_GetSensorStates(uint16_t *sensor_values);
float BinaryLineFollower_CalculateError(uint16_t *sensor_values);
int8_t Binary_GetLinePosition(uint16_t *sensor_values);

#endif