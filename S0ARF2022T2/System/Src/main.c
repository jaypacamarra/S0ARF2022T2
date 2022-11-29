#include "stm32L0x0.h"
#include "DEV_Battery.h"
#include "DEV_Sensor.h"
#include "DEV_NRF24.h"

int main(void)
{

	/* Init battery monitoring */

	/* Init GPIO sensing of reed switch */
	DEV_Sensor_Init();

	/* Init NRF24 handle */


	for(;;) {

		/* Sleep */
	}
}
