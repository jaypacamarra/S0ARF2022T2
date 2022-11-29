/*
 * STM32L0x0_gpio_driver.c
 *
 *  Created on: Nov. 26, 2022
 *      Author: jay
 */

#include "STM32L0x0_gpio_driver.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI) {
	if(pGPIOx == GPIOA) {
		if(ENorDI == ENABLE) {
			GPIOA_PCLK_EN();
		}
		else {
			GPIOA_PCLK_DI();
		}
	}
	else if(pGPIOx == GPIOB) {
		if(ENorDI == ENABLE) {
			GPIOB_PCLK_EN();
		}
		else {
			GPIOB_PCLK_DI();
		}
	}
	else if(pGPIOx == GPIOC) {
		if(ENorDI == ENABLE) {
			GPIOC_PCLK_EN();
		}
		else {
			GPIOC_PCLK_DI();
		}
	}
	else if(pGPIOx == GPIOD) {
		if(ENorDI == ENABLE) {
			GPIOD_PCLK_EN();
		}
		else {
			GPIOD_PCLK_DI();
		}
	}
	else if(pGPIOx == GPIOE) {
		if(ENorDI == ENABLE) {
			GPIOE_PCLK_EN();
		}
		else {
			GPIOE_PCLK_DI();
		}
	}
	else if(pGPIOx == GPIOH) {
		if(ENorDI == ENABLE) {
			GPIOH_PCLK_EN();
		}
		else {
			GPIOH_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - Initializes the GPIO
 *
 * @param[in]         - GPIO handle
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	uint32_t temp = 0;

	/* 1. Configure the mode of the GPIO pin */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* clear bits first */
		pGPIOHandle->pGPIOx->MODER |= temp;	/* set GPIO mode */
	}
	else {
		// code this part later (interrupt)
	}
	temp = 0;

	/* 2. Configure the speed */
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* clear bits first */
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;	/* set GPIO speed */
	temp = 0;

	/* 3. Configure the PuPd settings */
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* clear bits first */
	pGPIOHandle->pGPIOx->PUPDR |= temp;		/* set GPIO pupd */
	temp = 0;

	/* 4. Configure the output type */
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* clear bits first */
	pGPIOHandle->pGPIOx->OTYPER |= temp;	/* set GPIO output type */
	temp = 0;

	/* 5. Configure the alternate functionality */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_AltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->AFRL &= ~(0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* clear bits first */
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7) {
			pGPIOHandle->pGPIOx->AFRL |= temp;	/* set alternate function in LOW register */
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber >= 8) {
			pGPIOHandle->pGPIOx->AFRH |= temp;	/* set alternate function in HIGH register */
		}
		temp = 0;
	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - De-initializes the GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if(pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - Returns digital input data of a GPIO pin
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - pin number of the gpio port
 *
 * @return            - GPIO pin status, 1 = HIGH, 0 = LOW
 *
 * @Note              -
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	return (pGPIOx->IDR & (1 << PinNumber)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - Returns digital input data of a GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 *
 * @return            - GPIO port status
 *
 * @Note              -
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	return (uint16_t)pGPIOx->IDR;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - Writes Value to GPIO port/PinNumber
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - pin number of the gpio port
 * @param[in]         - value to write
 *
 * @return            - none
 *
 * @Note              -
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
	if(Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - Writes Value to GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - value to write to gpio port
 *
 * @return            - none
 *
 * @Note              -
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - Toggles the GPIO port/PinNumber
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - pin number of the gpio port
 *
 * @return            -
 *
 * @Note              -
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - Configure interrupt in NVIC set enable registers
 *
 * @param[in]         - IRQ number
 * @param[in]         - Enable or disable GPIO interrupt
 *
 * @return            - none
 *
 * @Note              -
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
		if(IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}
	}
	else {
		if(IRQNumber <= 31) {
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - Configures IRQ priority based on priority number and priority
 *
 * @param[in]         - IRQ number
 * @param[in]         - IRQ priority number
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - Clears the exti pending register to the corresponding pin number
 *
 * @param[in]         - GPIO pin number
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}

}

