/*
 * it_handlers.c
 *
 *
 * This source file contain all interrupt service routines
 * for user application
 *
 *
 *  Created on: Nov. 29, 2022
 *      Author: jay
 */

#include "stm32L0x0.h"
#include "DEV_Battery.h"

void EXTI0_1_IRQHandler(void) {
	GPIO_IRQHandling(GPIO_PIN_NO_1);
}

void LPTIM1_IRQHandler(void) {
	/* LPTIM IRQ Handling */
	LPTIM_IRQHandler();

	static uint8_t LPTIM_sec_counter;	/* keep track of seconds counted by LPTIM1 */
	LPTIM_sec_counter++;

	if(30 == LPTIM_sec_counter) {
		/* Turn on battery monitoring power */
		GPIO_WriteToOutputPin(DEV_BATTERY_MONITOR_PWR_PORT, DEV_BATTERY_MONITOR_PWR_PIN, GPIO_PIN_SET);

		/* Sample battery monitoring pin PA */
		uint8_t BattIsLow = GPIO_ReadFromInputPin(DEV_BATTERY_MONITOR_PORT, DEV_BATTERY_MONITOR_PIN);

		/* Turn off battery monitoring power */
		GPIO_WriteToOutputPin(DEV_BATTERY_MONITOR_PWR_PORT, DEV_BATTERY_MONITOR_PWR_PIN, GPIO_PIN_RESET);

		/* Send battery is low alert via NRF24 */
		if(BattIsLow) {

		}

		LPTIM_sec_counter = 0;
	}
}
