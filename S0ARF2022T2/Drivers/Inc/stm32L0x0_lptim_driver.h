/*
 * stm32l0x0_lptim_driver.h
 *
 *  Created on: Nov. 26, 2022
 *      Author: jay
 */

#ifndef STM32L0X0_LPTIM_DRIVER_H_
#define STM32L0X0_LPTIM_DRIVER_H_

#include "stm32L0x0.h"

/*
 * LPTIM configuration structure definition
 */
typedef struct {
	uint8_t 	LPTIM_Encoder;			/* Encoder mode enable, @LPTIM_Encoder 		*/
	uint8_t 	LPTIM_Interrupt_Mode;	/* Interrupt modes, @LPTIM_Int_Modes 		*/
	uint8_t 	LPTIM_Prescaler;		/* Prescaler division config, @LPTIM_Presc 	*/
	uint8_t 	LPTIM_CLK_SEL;			/* Clock select, @LPTIM_Clock_Sources		*/
	uint8_t 	LPTIM_Start_Mode;		/* LPTIM start modes, @LPTIM_start_modes	*/
	uint16_t 	LPTIM_CMP;				/* LPTIM compare value						*/
	uint16_t 	LPTIM_ARR;				/* LPTIM auto reload value					*/
}LPTIM_TIM_Config_t;

/*
 * LPTIM handle structure definition
 */
typedef struct {
	LPTIM_RegDef_t 		*pGPIOx;
	LPTIM_TIM_Config_t 	lptimConfig;
}LPTIM_Handle_t;

/*
 * Encoder mode configure LPTIM_CFGR register, @LPTIM_Encoder
 */
#define LPTIM_MODE_ENCODER_EN		(1U)
#define LPTIM_MODE_ENCODER_DI		(0U)

/*
 * Interrupt modes configure LPTIM_IER register, @LPTIM_Int_Modes
 */
#define LPTIM_INT_DOWN			(1 << 6)	/* interrupt on change direction to DOWN 		*/
#define LPTIM_INT_UP			(1 << 5)	/* interrupt on change direction to UP 			*/
#define LPTIM_INT_ARROK			(1 << 4)	/* interrupt on auto reload register update ok 	*/
#define LPTIM_INT_CMPOK			(1 << 3)	/* interrupt on compare register update ok 		*/
#define LPTIM_INT_EXTTRIG		(1 << 2)	/* interrupt on external trigger 				*/
#define LPTIM_INT_ARRM			(1 << 1)	/* interrupt on auto reload match 				*/
#define LPTIM_INT_CMPM			(1 << 0)	/* interrupt on compare match 					*/

/*
 * Prescaler possible values configure LPTIM_CFGR register, @LPTIM_Presc
 */
#define LPTIM_PRESC_DIV_1		(0U)
#define LPTIM_PRESC_DIV_2		(1U)
#define LPTIM_PRESC_DIV_4		(2U)
#define LPTIM_PRESC_DIV_8		(3U)
#define LPTIM_PRESC_DIV_16		(4U)
#define LPTIM_PRESC_DIV_32		(5U)
#define LPTIM_PRESC_DIV_64		(6U)
#define LPTIM_PRESC_DIV_128		(7U)

/*
 * Clock sources configure LPTIM_CFGR register, @LPTIM_Clock_Sources
 */
#define LPTIM_CLK_SRC_INT		(0U)
#define LPTIM_CLK_SRC_EXT		(1U)

/*
 * LPTIM start modes configure LPTIM_CFGR register, @LPTIM_start_modes
 */
#define LPTIM_TIM_START_CONT	(1 << 2)
#define LPTIM_TIM_START_SING	(1 << 1)

/*
 * LPTIM enable & disable macro configure LPTIM_CR register
 */
#define LPTIM_TIM_EN	(1 << 0)
#define LPTIM_TIM_DI	~(1 << 0)

/*
 * LPTIM driver function prototypes
 */
void LPTIM_Timer_Init(LPTIM_Handle_t *pLPTIMx);
void LPTIM_Timer_Stop(LPTIM_Handle_t *pLPTIMx);
void LPTIM_IRQHandler(void);

#endif /* STM32L0X0_LPTIM_DRIVER_H_ */
