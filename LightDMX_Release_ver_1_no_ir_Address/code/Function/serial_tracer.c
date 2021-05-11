/*
 * serial_tracer.c
 *
 *  Created on: 12. 3. 2013
 *      Author: honza
 */
	

#include "stm32f10x.h"
#include "serial_tracer.h"
#include <stdio.h >
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>


void Usart_Init(int baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB |	RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
}

void Uart1WriteString(const char *txtString)
{
	while((*txtString)!='\0')
	{
	 while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) {};   
    USART_SendData(USART3,(uint8_t)*txtString++); 
	}
}

void printf2(const char *format,...){
    char temp[200];
    __va_list arg;
    va_start (arg, format);
    vsprintf (temp, format, arg);
    va_end (arg);
    Uart1WriteString(temp);
}

