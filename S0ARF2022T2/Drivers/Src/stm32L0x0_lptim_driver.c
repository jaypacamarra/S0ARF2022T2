/*
 * stm32L0x0_lptim_driver.c
 *
 *  Created on: Nov. 26, 2022
 *      Author: jay
 */

#include "stm32L0x0_lptim_driver.h"

/*********************************************************************
 * @fn      		  - LPTIM_Timer_Init
 *
 * @brief             - Initializes and starts LPTIM
 *
 * @param[in]         - pointer to a LPTIM handle
 * @param[in]         - none
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void LPTIM_Timer_Init(LPTIM_Handle_t *pLPTIMx) {
	LPTIM->ARR = pLPTIMx->lptimConfig.LPTIM_ARR;
	LPTIM->CFGR |= (pLPTIMx->lptimConfig.LPTIM_CLK_SEL << 0);
	LPTIM->IER |= pLPTIMx->lptimConfig.LPTIM_Interrupt_Mode;
	LPTIM->CFGR |= pLPTIMx->lptimConfig.LPTIM_Prescaler;
	LPTIM->CR |= LPTIM_TIM_EN;
	LPTIM->CR |= pLPTIMx->lptimConfig.LPTIM_Start_Mode;
}

/*********************************************************************
 * @fn      		  - LPTIM_Timer_Stop
 *
 * @brief             - Stops the LPTIM timer
 *
 * @param[in]         - pointer to a LPTIM handle
 * @param[in]         - none
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void LPTIM_Timer_Stop(LPTIM_Handle_t *pLPTIMx) {
	LPTIM->CR &= LPTIM_TIM_DI;
}

/*********************************************************************
 * @fn      		  - LPTIM_IRQHandler
 *
 * @brief             - Handles LPTIM IRQ by clearing all interrupt flags
 *
 * @param[in]         - none
 * @param[in]         - none
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void LPTIM_IRQHandler(void) {
	uint8_t clear_all_lptim_int_mask = ~(0x7F);
	LPTIM->ICR &= clear_all_lptim_int_mask;
}
