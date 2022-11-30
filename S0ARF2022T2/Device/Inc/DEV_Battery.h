/*
 * Battery.h
 *
 *  Created on: Nov. 26, 2022
 *      Author: jay
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include "stm32L0x0.h"

#define DEV_BATTERY_MONITOR_PORT		GPIOA
#define DEV_BATTERY_MONITOR_PIN			GPIO_PIN_NO_9
#define DEV_BATTERY_MONITOR_PWR_PORT	GPIOA
#define DEV_BATTERY_MONITOR_PWR_PIN		GPIO_PIN_NO_2

void DEV_Battery_Monitor_Init(void);

#endif /* BATTERY_H_ */
