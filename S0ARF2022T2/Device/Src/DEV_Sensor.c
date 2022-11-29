/*
 * Sensor.c
 *
 *  Created on: Nov. 26, 2022
 *      Author: jay
 */

#include "DEV_Sensor.h"

/*********************************************************************
 * @fn      		  - DEV_Sensor_Init
 *
 * @brief             - Initialize the GPIO pin that will read the reed sensor
 *
 * @param[in]         - none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 */
void DEV_Sensor_Init(void) {
	GPIO_Handle_t ReedSensor;
	ReedSensor.pGPIOx = DEV_SENSOR_PORT;
	ReedSensor.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	ReedSensor.GPIO_PinConfig.GPIO_PinNumber = DEV_SENSOR_PIN;
	ReedSensor.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	ReedSensor.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(DEV_SENSOR_PORT, ENABLE);
	GPIO_Init(&ReedSensor);
}

/*********************************************************************
 * @fn      		  - DEV_Sensor_Read
 *
 * @brief             - Returns sensor value from reed switch GPIO
 *
 * @param[in]         - GPIO peripheral base address
 * @param[in]         - GPIO pin number
 *
 * @return            - sensor value
 *
 * @Note              - none
 */
uint8_t DEV_Sensor_Read(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	return (pGPIOx->IDR & (1 << PinNumber)) > 1 ? GPIO_PIN_SET : GPIO_PIN_RESET;
}


