#include "DEV_Battery.h"
#include "DEV_Sensor.h"
#include "DEV_NRF24.h"

void SYS_CLK_Init(void);

int main(void)
{

	/* Setup system clock with 16Mhz HSI*/
	SYS_CLK_Init();

	/* Init battery monitoring */
	DEV_Battery_Monitor_Init();

	/* Init GPIO sensing of reed switch */
	DEV_Sensor_Init();

	/* Init NRF24 handle */


	for(;;) {

		/* Sleep */
	}
}

/*********************************************************************
 * @fn      		  - SYS_CLK_Init
 *
 * @brief             - Initializes system clock source as HSI 16 Mhz
 *
 * @param[in]         - none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 */
void SYS_CLK_Init(void) {
	#define HSI16ON		BIT0
	#define HSI16RDY	(RCC->CR & (1 << BIT2))
	RCC->CR = 0;
	RCC->CR |= HSI16ON;
	while(!HSI16RDY);	/* wait for HSI to stabilize */
}


