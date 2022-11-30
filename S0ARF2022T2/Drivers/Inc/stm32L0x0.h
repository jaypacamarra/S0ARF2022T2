/*
 * STM32L0x0.h
 *
 *  Created on: Nov. 26, 2022
 *      Author: jay
 */

#ifndef STM32L0X0_H_
#define STM32L0X0_H_

#include <stdint.h>



#define __vo volatile

/********************************************************
 * (START) Processor specific details
 *********************************************************/
/*
 * ARM Cortex Mx processor NVIC ISERx register addresses
 */
#define NVIC_ISER0			((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t *)0xE000E10C)

/*
 * ARM Cortex Mx processor NVIC ICERx register addresses
 */
#define NVIC_ICER0			((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1			((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2			((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3			((__vo uint32_t *)0xE000E18C)

/*
 * NVIC PR base address
 */
#define NVIC_PR_BASE_ADDR	((__vo uint32_t *)0xE000E400)

/*
 * Number of PR bits implemented for ARM cortex-Mx processor
 */
#define NO_PR_BITS_IMPLEMENTED		(4U)

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 */
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4			84
#define IRQ_NO_I2C1_EV     	31
#define IRQ_NO_I2C1_ER     	32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    	0
#define NVIC_IRQ_PRI1    	1
#define NVIC_IRQ_PRI2    	2
#define NVIC_IRQ_PRI3    	3
#define NVIC_IRQ_PRI4    	4
#define NVIC_IRQ_PRI5    	5
#define NVIC_IRQ_PRI6    	6
#define NVIC_IRQ_PRI7    	7
#define NVIC_IRQ_PRI8    	8
#define NVIC_IRQ_PRI9    	9
#define NVIC_IRQ_PRI10    	10
#define NVIC_IRQ_PRI11    	11
#define NVIC_IRQ_PRI12    	12
#define NVIC_IRQ_PRI13    	13
#define NVIC_IRQ_PRI14    	14
#define NVIC_IRQ_PRI15    	15

/*
 * General macros
 */

#define ENABLE			(1U)
#define DISABLE			(0U)
#define True			ENABLE
#define	False			DISABLE
#define SET				ENABLE
#define RESET			DISABLE
#define	GPIO_PIN_SET	SET
#define	GPIO_PIN_RESET	RESET

#define BIT0			(0U)
#define BIT1			(1U)
#define	BIT2			(2U)
#define	BIT3			(3U)
#define	BIT4			(4U)
#define	BIT5			(5U)
#define	BIT6			(6U)
#define	BIT7			(7U)
#define	BIT8			(8U)
#define	BIT9			(9U)
#define	BIT10			(10U)
#define	BIT11			(11U)
#define	BIT12			(12U)
#define	BIT13			(13U)
#define	BIT14			(14U)
#define	BIT15			(15U)

/*
 * Flash and SRAM base addresses
 */

#define FLASH_BASEADDR				0x08000000U			/* start address of flash memory or main memory */
#define SRAM1_BASEADDR				0x20000000U			/* start address of SRAM1 						*/
#define USR_OPTION_BYTES_BASEADDR	0x1FF80000U			/* start address of user option bytes area 		*/
#define FCTRY_OPTION_BYTES_BASEADDR	0x1FF80020U			/* start address of factory option bytes area 	*/
#define ROM_BASEADDR				0x1FF00000U			/* start address of ROM or system memory 		*/
#define SRAM						SRAM1_BASEADDR		/* start address of SRAM 						*/

/*
 * AHBx, APBx, IOPORT bus peripheral base addresses
 */

#define PERIPH_BASEADDR				0x40000000U			/* start address of peripherals */
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR		/* start address of APB1 		*/
#define APB2PERIPH_BASEADDR			0x40010000U			/* start address of APB2 		*/
#define AHBPERIPH_BASEADDR			0x40020000U			/* start address of AHB 		*/
#define IOPORT_BASEADDR				0x50000000U			/* start address of IOPORT		*/

/*
 * IOPORT peripheral base addresses
 */

#define GPIOA_BASEADDR				(IOPORT_BASEADDR + 0x0000U)			/* start address of GPIOA 			*/
#define GPIOB_BASEADDR				(IOPORT_BASEADDR + 0x0400U)			/* start address of GPIOB 			*/
#define GPIOC_BASEADDR				(IOPORT_BASEADDR + 0x0800U)			/* start address of GPIOC 			*/
#define GPIOD_BASEADDR				(IOPORT_BASEADDR + 0x0C00U)			/* start address of GPIOD 			*/
#define GPIOE_BASEADDR				(IOPORT_BASEADDR + 0x1000U)			/* start address of GPIOE 			*/
#define GPIOH_BASEADDR				(IOPORT_BASEADDR + 0x1C00U)			/* start address of GPIOH 			*/

/*
 *  AHB peripheral base address
 */

#define CRC_BASEADDR				(AHBPERIPH_BASEADDR + 0x3000U)		/* start address of CRC 				*/
#define RCC_BASEADDR				(AHBPERIPH_BASEADDR + 0x1000U)		/* start address of RCC 				*/
#define FLASH_REG_BASEADDR			(AHBPERIPH_BASEADDR + 0x2000U)		/* start address of Flash register map	*/
#define DMA1_BASEADDR				(AHBPERIPH_BASEADDR + 0x0000U)		/* start address of DMA1 				*/

/*
 * APB1 peripheral base addresses
 */

#define TIM2_BASEADDR				(APB1PERIPH_BASEADDR + 0x0000U)		/* start address of TIM2	*/
#define RTC_BASEADDR				(APB1PERIPH_BASEADDR + 0x2800U)		/* start address of RTC		*/
#define WWDG_BASEADDR				(APB1PERIPH_BASEADDR + 0x2C00U)		/* start address of WWDG	*/
#define IWDG_BASEADDR				(APB1PERIPH_BASEADDR + 0x2C00U)		/* start address of IWDG	*/
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400U)		/* start address of USART2	*/
#define LPUART1_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800U)		/* start address of LPUART1	*/
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400U)		/* start address of I2C1 	*/
#define PWR_BASEADDR				(APB1PERIPH_BASEADDR + 0x7000U)		/* start address of PWR		*/
#define LPTIM1_BASEADDR				(APB1PERIPH_BASEADDR + 0x7C00U)		/* start address of LPTIM1	*/

/*
 * APB2 peripheral base addresses
 */

#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800U)		/* start address of SYSCFG 	*/
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x0400U)		/* start address of EXTI 	*/
#define TIM21_BASEADDR				(APB2PERIPH_BASEADDR + 0x0800U)		/* start address of TIM21	*/
#define TIM22_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400U)		/* start address of TIM21	*/
#define FIREWALL_BASEADDR			(APB2PERIPH_BASEADDR + 0x1C00U)		/* start address of Firewall*/
#define ADC1_BASEADDR				(APB2PERIPH_BASEADDR + 0x2400U)		/* start address of ADC		*/
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000U)		/* start address of SPI1 	*/
#define DBG_BASEADDR				(APB2PERIPH_BASEADDR + 0x5800U)		/* start address of DBG 	*/


/************************* PERIPHERAL REGISTER DEFINITION STRUCTURES ************************ */

/*
 * Note:
 * Registers and peripherals are MCU specific, please refer to your MCU datasheet
 */


/*
 * GPIO register structure definition
 * There is a 4 byte address offset between each member
 */

typedef struct {
	__vo uint32_t MODER;	/* GPIO port mode register 					*/
	__vo uint32_t OTYPER;	/* GPIO port output type register 			*/
	__vo uint32_t OSPEEDR;	/* GPIO port output speed register 			*/
	__vo uint32_t PUPDR;	/* GPIO port pull-up/pull-down register 	*/
	__vo uint32_t IDR;		/* GPIO port input data register 			*/
	__vo uint32_t ODR;		/* GPIO port output data register 			*/
	__vo uint32_t BSRR;		/* GPIO port bit set/reset register 		*/
	__vo uint32_t LCKR;		/* GPIO port configuration lock register 	*/
	__vo uint32_t AFRL;		/* GPIO alternate function low register 	*/
	__vo uint32_t AFRH;		/* GPIO alternate function high register 	*/
	__vo uint32_t BRR;		/* GPIO port bit reset register				*/

}GPIO_RegDef_t;

/*
 *  RCC register structure definition
 */

typedef struct {
	__vo uint32_t CR;			/* RCC clock control register 									*/
	__vo uint32_t ICSCR;		/* Internal clock sources calibration register					*/
	uint32_t RESERVED0;			/* Reserved 0x1C 												*/
	__vo uint32_t CFGR;			/* RCC clock configuration register 							*/
	__vo uint32_t CIER;			/* RCC clock interrupt enable register 							*/
	__vo uint32_t CIFR;			/* Clock interrupt flag register 								*/
	__vo uint32_t CICR;			/* CLock interrupt clear register								*/
	__vo uint32_t IOPRSTR;		/* GPIO reset register											*/
	__vo uint32_t AHBRSTR;		/* AHB peripheral reset register 								*/
	__vo uint32_t APB2RSTR;		/* RCC APB2 peripheral reset register 							*/
	__vo uint32_t APB1RSTR;		/* RCC APB1 peripheral reset register 							*/
	__vo uint32_t IOPENR;		/* GPIO clock enable register 									*/
	__vo uint32_t AHBENR;		/* AHB peripheral clock enable register 						*/
	__vo uint32_t APB2ENR;		/* APB2 peripheral clock enable register 						*/
	__vo uint32_t APB1ENR;		/* APB1 peripheral clock enable register 						*/
	__vo uint32_t IOPSMENR;		/* GPIO clock enable in Sleep mode register 					*/
	__vo uint32_t AHBSMENR;		/* AHB peripheral clock enable in Sleep mode register 			*/
	__vo uint32_t APB2SMENR;	/* APB2 peripheral clock enable in Sleep mode register 			*/
	__vo uint32_t APB1SMENR;	/* APB2 peripheral clock enable in Sleep mode register			*/
	__vo uint32_t CCIPR;		/* Clock configuration register 								*/
	__vo uint32_t CSR;			/* Control/status register 										*/

}RCC_RegDef_t;

/*
 *  EXTI register structure definition
 */
typedef struct {
	__vo uint32_t IMR;		/* EXTI Interrupt mask register 			*/
	__vo uint32_t EMR;		/* EXTI Event mask register 				*/
	__vo uint32_t RTSR;		/* EXTI Rising trigger selection register 	*/
	__vo uint32_t FTSR;		/* EXTI Falling trigger selection register 	*/
	__vo uint32_t SWIER;	/* EXTI Software interrupt event register	*/
	__vo uint32_t PR;		/* EXTI Pending register					*/
}EXTI_RegDef_t;

/*
 * SYSCFG register structure definition
 */
typedef struct {
	__vo uint32_t CFGR1;		/* SYSCFG memory remap register 						*/
	__vo uint32_t CFGR2;		/* SYSCFG peripheral mode configuration register 		*/
	__vo uint32_t EXTICR[4];	/* SYSCFG external interrupt configuration registers 	*/
	uint32_t RESERVED[2];		/* Reserved 0x18 - 0x1C									*/
	__vo uint32_t CFGR3;		/* Reference control and status register 				*/
}SYSCFG_RegDef_t;

/*
 * LPTIM register structure definition
 */
typedef struct {
	__vo uint32_t ISR;			/* LPTIM interrupt and status register 	*/
	__vo uint32_t ICR;			/* LPTIM interrupt clear register 		*/
	__vo uint32_t IER;			/* LPTIM interrupt enable register 		*/
	__vo uint32_t CFGR;			/* LPTIM configuration register 		*/
	__vo uint32_t CR;			/* LPTIM control register 				*/
	__vo uint32_t CMP;			/* LPTIM compare register 				*/
	__vo uint32_t ARR;			/* LPTIM auto reload register 			*/
	__vo uint32_t CNT;			/* LPTIM counter register 				*/
}LPTIM_RegDef_t;

/*
 * Peripheral definitions (peripheral base address typecasted to [XXXX_RegDeg_t])
 */

#define GPIOA		((GPIO_RegDef_t *)GPIOA_BASEADDR)	/* Use to access GPIOA peripheral */
#define GPIOB		((GPIO_RegDef_t *)GPIOB_BASEADDR)	/* Use to access GPIOB peripheral */
#define GPIOC		((GPIO_RegDef_t *)GPIOC_BASEADDR)	/* Use to access GPIOC peripheral */
#define GPIOD		((GPIO_RegDef_t *)GPIOD_BASEADDR)	/* Use to access GPIOD peripheral */
#define GPIOE		((GPIO_RegDef_t *)GPIOE_BASEADDR)	/* Use to access GPIOE peripheral */
#define GPIOH		((GPIO_RegDef_t *)GPIOH_BASEADDR)	/* Use to access GPIOH peripheral */
#define RCC			((RCC_RegDef_t *)RCC_BASEADDR)		/* Use to access RCC   peripheral */
#define EXTI		((EXTI_RegDef_t *)EXTI_BASEADDR)	/* Use to access EXTI peripheral  */
#define SYSCFG		((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)/* Use to access SYSCFG peripheral*/
#define LPTIM		((LPTIM_RegDef_t *)LPTIM1_BASEADDR) /* Use to access LPTIM peripherals*/


/************************* CLOCK ENABLE & DISABLE MACROS ************************ */

/*
 * GPIOx clock Enable macros
 */
#define GPIOA_PCLK_EN()		(RCC->IOPENR |= (1 << 0))	/* Enable peripheral clock for GPIOA */
#define GPIOB_PCLK_EN()		(RCC->IOPENR |= (1 << 1))	/* Enable peripheral clock for GPIOB */
#define GPIOC_PCLK_EN()		(RCC->IOPENR |= (1 << 2))	/* Enable peripheral clock for GPIOC */
#define GPIOD_PCLK_EN()		(RCC->IOPENR |= (1 << 3))	/* Enable peripheral clock for GPIOD */
#define GPIOE_PCLK_EN()		(RCC->IOPENR |= (1 << 4))	/* Enable peripheral clock for GPIOE */
#define GPIOH_PCLK_EN()		(RCC->IOPENR |= (1 << 7))	/* Enable peripheral clock for GPIOH */

/*
 * I2Cx clock Enable macros
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))	/* Enable peripheral clock for I2C1 */

/*
 * SPIx clock Enable macros
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))	/* Enable peripheral clock for SPI1 */

/*
 * UART/USART clock Enable macros
 */
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))	/* Enable peripheral clock for USART2 	*/
#define LPUART1_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))	/* Enable peripheral clock for LPUART1 	*/

/*
 * SYSCFG clock Enable macros
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 0))	/* Enable peripheral clock for SYSCFG */




/*
 * GPIOx clock Disable macros
 */
#define GPIOA_PCLK_DI()		(RCC->IOPENR &= ~(1 << 0))		/* Disable peripheral clock for GPIOA */
#define GPIOB_PCLK_DI()		(RCC->IOPENR &= ~(1 << 1))		/* Disable peripheral clock for GPIOB */
#define GPIOC_PCLK_DI()		(RCC->IOPENR &= ~(1 << 2))		/* Disable peripheral clock for GPIOC */
#define GPIOD_PCLK_DI()		(RCC->IOPENR &= ~(1 << 3))		/* Disable peripheral clock for GPIOD */
#define GPIOE_PCLK_DI()		(RCC->IOPENR &= ~(1 << 4))		/* Disable peripheral clock for GPIOE */
#define GPIOH_PCLK_DI()		(RCC->IOPENR &= ~(1 << 7))		/* Disable peripheral clock for GPIOH */

/*
 * I2Cx clock Disable macros
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))	/* Disable peripheral clock for I2C1 */

/*
 * SPIx clock Disable macros
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))	/* Disable peripheral clock for SPI1 */

/*
 * UART/USART clock Disable macros
 */
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))	/* Disable peripheral clock for USART2 	*/
#define LPUART1_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))	/* Disable peripheral clock for LPUART1	*/

/*
 * SYSCFG clock Disable macros
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 0))	/* Disable peripheral clock for SYSCFG */

/*
 * GPIO Reset macros
 */

#define GPIOA_REG_RESET()	do{RCC->IOPRSTR |= (1<<0); RCC->IOPRSTR &= ~(1<<0); }while(0)
#define GPIOB_REG_RESET()	do{RCC->IOPRSTR |= (1<<1); RCC->IOPRSTR &= ~(1<<1); }while(0)
#define GPIOC_REG_RESET()	do{RCC->IOPRSTR |= (1<<2); RCC->IOPRSTR &= ~(1<<2); }while(0)
#define GPIOD_REG_RESET()	do{RCC->IOPRSTR |= (1<<3); RCC->IOPRSTR &= ~(1<<3); }while(0)
#define GPIOE_REG_RESET()	do{RCC->IOPRSTR |= (1<<4); RCC->IOPRSTR &= ~(1<<4); }while(0)
#define GPIOH_REG_RESET()	do{RCC->IOPRSTR |= (1<<7); RCC->IOPRSTR &= ~(1<<7); }while(0)


/*
 * Macro to return port code to configure SYSCFG peripheral's EXTICR register
 * Returns port code [0 - 7] from a given GPIO port address
 */
#define GPIO_BASEADDR_TO_CODE(x)		( (x == GPIOA) ? 0 : \
										  (x == GPIOB) ? 1 : \
										  (x == GPIOC) ? 2 : \
										  (x == GPIOD) ? 3 : \
									      (x == GPIOE) ? 4 : \
									      (x == GPIOH) ? 7 : 0 )


#include "stm32L0x0_gpio_driver.h"
#include "stm32L0x0_adc_driver.h"
#include "stm32L0x0_spi_driver.h"
#include "stm32L0x0_lptim_driver.h"
#include "stm32L0x0_interrupt_driver.h"


#endif /* STM32L0X0_H_ */
