/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			24C02.h
** Descriptions:		24C02 ²Ù×÷º¯Êý¿â 
**
**------------------------------------------------------------------------------------------------------
** Created by:			AVRman
** Created date:		2010-10-29
** Version:				1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:	
** Version:
** Descriptions:		
********************************************************************************************************/
#ifndef __24C02_H
#define __24C02_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "serial_tracer.h"
 
/* Private define ------------------------------------------------------------*/
#define AT24C01A		/* 24C01A,I2CÊ±ÐòºÍÍùºóµÄ24C02Ò»Ñù */
//#define AT24C01		/* 24C01,I2CµÄÊ±ÐòºÍÆÕÍ¨µÄÓÐµã²»Í¬ */

#define ADDR_24LC02		0xAE
#define I2C_PAGESIZE	8		/* 24C01/01AÒ³»º³åÊÇ4¸ö */
#define DEBUG_EPPROM

#ifndef DEBUG_EPPROM
#define DEBUG(x)
#else
#define DEBUG	printf2
#endif

/* Private function prototypes -----------------------------------------------*/
void I2C_Configuration(void);
uint8_t I2C_WriteOneByte(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint16_t addr,uint8_t value);
uint8_t I2C_Read(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint16_t addr,uint8_t *buf,uint16_t num);
uint8_t I2C_Write(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint16_t addr,uint8_t *buf,uint16_t num);

uint8_t EPPROM_Read(uint16_t addr,uint8_t *buf,uint16_t num);
uint8_t EPPROM_Write(uint16_t addr,uint8_t *buf,uint16_t num);
void Test_ReadWriteEEp(void);
#endif 

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

