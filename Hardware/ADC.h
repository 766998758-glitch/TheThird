#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"

#define IR_SENSOR_COUNT 4


void ADC1_Module_Init(void);
void ADC_ReadAllSensors(uint16_t *sensor_values);
uint16_t ADC_ReadSingleSensor(uint8_t channel);

#endif
