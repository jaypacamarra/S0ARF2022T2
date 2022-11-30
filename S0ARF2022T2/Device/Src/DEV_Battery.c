/*
 * Battery.c
 *
 *  Created on: Nov. 26, 2022
 *      Author: jay
 */

#include "DEV_Battery.h"

/*********************************************************************
 * @fn      		  - DEV_Battery_Monitor_Init
 *
 * @brief             - Initialize the GPIO port and pin for monitoring battery
 * 						Initialize the GPIO port and pin for powering battery monitor
 * 						Initialize the LPTIM peripheral
 *
 * @param[in]         - none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 */
void DEV_Battery_Monitor_Init(void) {
	/* GPIO pin configured as input pin for battery monitoring */
	GPIO_Handle_t BatteryMonitor;
	BatteryMonitor.pGPIOx = DEV_BATTERY_MONITOR_PORT;
	BatteryMonitor.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	BatteryMonitor.GPIO_PinConfig.GPIO_PinNumber = DEV_BATTERY_MONITOR_PIN;
	BatteryMonitor.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	BatteryMonitor.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(DEV_BATTERY_MONITOR_PORT, ENABLE);
	GPIO_Init(&BatteryMonitor);

	/* GPIO pin configured as output push-pull to deliver power to battery monitoring */
	GPIO_Handle_t BatteryMonitorPwr;
	BatteryMonitorPwr.pGPIOx = DEV_BATTERY_MONITOR_PWR_PORT;
	BatteryMonitorPwr.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	BatteryMonitorPwr.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	BatteryMonitorPwr.GPIO_PinConfig.GPIO_PinNumber = DEV_BATTERY_MONITOR_PWR_PIN;
	BatteryMonitorPwr.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	BatteryMonitorPwr.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(DEV_BATTERY_MONITOR_PWR_PORT, ENABLE);
	GPIO_Init(&BatteryMonitorPwr);

	/* Init LPTIM to generate LPTIM1 interrupt every 1 second */
	LPTIM_Handle_t lptim1;
	lptim1.lptimConfig.LPTIM_CLK_SEL = LPTIM_CLK_SRC_INT;
	lptim1.lptimConfig.LPTIM_Encoder = LPTIM_MODE_ENCODER_DI;
	lptim1.lptimConfig.LPTIM_Prescaler = LPTIM_PRESC_DIV_16;	/* if system clock freq = 16 Mhz, output frequency is 1MHz */
	lptim1.lptimConfig.LPTIM_ARR = (2 ^ 16) - 1;				/* With prescaler of 16, ARR match every 1 second */
	lptim1.lptimConfig.LPTIM_Interrupt_Mode = LPTIM_INT_ARRM;
	lptim1.lptimConfig.LPTIM_Start_Mode = LPTIM_TIM_START_CONT;
	LPTIM_Timer_Init(&lptim1);
}

