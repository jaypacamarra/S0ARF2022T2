/*
 * Sensor.h
 *
 *  Created on: Nov. 26, 2022
 *      Author: jay
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "stm32L0x0.h"

#define DEV_SENSOR_PORT	GPIOA
#define DEV_SENSOR_PIN	GPIO_PIN_NO_10

void 	DEV_Sensor_Init	(void);
uint8_t DEV_Sensor_Read	(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

#endif /* SENSOR_H_ */
