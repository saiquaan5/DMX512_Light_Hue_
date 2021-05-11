/*
 * RTC_BKP.h
 *
 *  Created on: Nov 8, 2016
 *      Author: Phap.Ha
 */
	
	#ifndef RTC_BKP_H
	#define RTC_BKP_H
	
	#include "stm32f10x_bkp.h"
	#include "stm32f10x_rtc.h"
	#include "stm32f10x_rcc.h"
	
//	#define RTC_ENABLE
	
  typedef enum
	{
		DOT=0,DriverMode,
		MULX,MULY,MULZ,
		DVX,DVY,DVZ,
		EDM,FS,
		DLON,DLON1,DLOFF,DLOFF1,DLTOFF,DLTOFF1,CPX,CPX1,CPY,CPY1,CPZ,CPZ1,
		KV,KV1,CheckPass,CheckPass1,DeepSet,DeepSet1,License,
	}BKP_R_Enum;
	
	
	#define Licensedata 0xA1A2
	
	//#define WriteBKPdata16(x,y)		BKP_WriteBackupRegister(BKPDataReg[x],y)
	//#define WriteBKPdata32(x,y)		{ BKP_WriteBackupRegister(BKPDataReg[x],y>>16); BKP_WriteBackupRegister(BKPDataReg[x+1],y&0xffff);}
	//#define ReadBKPdata16(x)			BKP_ReadBackupRegister(BKPDataReg[x])
//	#define ReadBKPdata32(x)			((BKP_ReadBackupRegister(BKPDataReg[x])<<16)|(BKP_ReadBackupRegister(BKPDataReg[x+1])))
	
void WriteBKPdata16(uint16_t u16_BKR,uint16_t BKPD);
void WriteBKPdata32(uint16_t x ,uint32_t y);
uint16_t ReadBKPdata16(uint16_t x);
uint32_t ReadBKPdata32(uint16_t x);

	void RTC_BKP_Init(void);

	
	#endif