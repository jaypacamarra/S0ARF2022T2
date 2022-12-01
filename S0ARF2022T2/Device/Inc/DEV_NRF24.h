/*
 * NRF24.h
 *
 *  Created on: Nov. 26, 2022
 *      Author: jay
 */

#ifndef NRF24_H_
#define NRF24_H_

#include "stm32L0x0.h"

typedef struct {
	/* TODO */
}NRF24_Data_t;

void			NRF24_Send		(void);
NRF24_Data_t 	NRF24_Receive	(void);

#endif /* NRF24_H_ */
