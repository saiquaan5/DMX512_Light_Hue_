/*
 * RTC_BKP.c
 *
 *  Created on: Nov 8, 2016
 *      Author: Phap.Ha
 */
 
 #include "RTC_BKP.h"
 
 uint16_t BKPDataReg[42] =
  {
    BKP_DR1, BKP_DR2, BKP_DR3, BKP_DR4, BKP_DR5, BKP_DR6, BKP_DR7, BKP_DR8,
    BKP_DR9, BKP_DR10, BKP_DR11, BKP_DR12, BKP_DR13, BKP_DR14, BKP_DR15, BKP_DR16,
    BKP_DR17, BKP_DR18, BKP_DR19, BKP_DR20, BKP_DR21, BKP_DR22, BKP_DR23, BKP_DR24,
    BKP_DR25, BKP_DR26, BKP_DR27, BKP_DR28, BKP_DR29, BKP_DR30, BKP_DR31, BKP_DR32,
    BKP_DR33, BKP_DR34, BKP_DR35, BKP_DR36, BKP_DR37, BKP_DR38, BKP_DR39, BKP_DR40,
    BKP_DR41, BKP_DR42
  };  

void RTC_BKP_Init(void)
{
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
		PWR_BackupAccessCmd(ENABLE);
		//BKP_DeInit();
		#ifdef RTC_ENABLE
		RCC_LSEConfig(RCC_LSE_ON);
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
		BKP_ClearFlag();
		RCC_RTCCLKCmd(ENABLE);
		RTC_WaitForSynchro();
		RTC_WaitForLastTask();
//		RTC_ITConfig(RTC_IT_SEC, ENABLE);
//		RTC_WaitForLastTask();
		RTC_SetPrescaler(32767);
		RTC_WaitForLastTask();
		#endif
}
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  /* Enable the RTC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void WriteBKPdata16(uint16_t u16_BKR,uint16_t BKPD)
{
		BKP_WriteBackupRegister(BKPDataReg[u16_BKR],BKPD);
}
void WriteBKPdata32(uint16_t x ,uint32_t y)		
{ 
	BKP_WriteBackupRegister(BKPDataReg[x],y>>16); 
	BKP_WriteBackupRegister(BKPDataReg[x+1],y&0xffff);
}

uint16_t ReadBKPdata16(uint16_t x)
{
	return BKP_ReadBackupRegister(BKPDataReg[x]);
}
 
uint32_t ReadBKPdata32(uint16_t x)
{
	return (BKP_ReadBackupRegister(BKPDataReg[x])<<16)|(BKP_ReadBackupRegister(BKPDataReg[x+1]));
}
	
 
 