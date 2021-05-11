/*
 * dmx512_rec.h
 *
 *  Created on: 27.1.2013
 *      Author: honza
 */

#ifndef DMX512_REC_H_
#define DMX512_REC_H_

#include <string.h>
#include <stdint.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>

enum packet_type {
	DATA_PACKET = 0,
	TEST_PACKET,
};


void dmx512_rec_init(TIM_TypeDef *timx, USART_TypeDef *usartx);
void dmx512_rec_enable(int on);
void dmx512_init(void);

#endif /* DMX512_REC_H_ */
