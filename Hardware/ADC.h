#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"

#define IR_SENSOR_COUNT 4
#define BLACK_THRESHOLD 1500   // 黑线阈值（低于此值为黑线）
#define WHITE_THRESHOLD 2500   // 白区阈值（高于此值为白区）


void ADC1_Module_Init(void);  
void ADC_ReadAllSensors(uint16_t *sensor_values);
uint16_t ADC_ReadSingleSensor(uint8_t channel);
uint8_t GetLinePosition(uint16_t *sensor_values);
float BinaryLineFollower_CalculateError(uint16_t *sensor_values);

#endif