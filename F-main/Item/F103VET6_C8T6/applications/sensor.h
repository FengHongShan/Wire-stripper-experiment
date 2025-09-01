#ifndef SENSOR_H
#define SENSOR_H

#include "stm32f1xx.h"
#include "stdint.h"
#include "stm32f1xx_hal.h"
void Sensor_GPIO_Init(void);

#define ReadSensor HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)

#endif



