/****************************************Copyright (c)**************************************************                         
	
********************************************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "24C02.h"
#include <stdio.h>
#include <string.h>
/*******************************************************************************
* Function Name  : I2C_Configuration
* Description    : EEPROM
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void I2C_Configuration(void)
{
   I2C_InitTypeDef  I2C_InitStructure;
   GPIO_InitTypeDef  GPIO_InitStructure; 

   RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);

   /* Configure I2C2 pins: PB6->SCL and PB7->SDA */
   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
		
   I2C_DeInit(I2C1);
   I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
   I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
   I2C_InitStructure.I2C_OwnAddress1 = 0x30;
   I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
   I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
   I2C_InitStructure.I2C_ClockSpeed = 100000;  
    
   I2C_Cmd(I2C1, ENABLE);
   I2C_Init(I2C1, &I2C_InitStructure);

   I2C_AcknowledgeConfig(I2C1, ENABLE);
}
 
/*******************************************************************************
* Function Name  : I2C_delay
* Description    : 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void I2C_delay(uint16_t cnt)
{
	while(cnt--);
}

/*******************************************************************************
* Function Name  : I2C_AcknowledgePolling
* Description    : 
* Input          : 
*                  
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void I2C_AcknowledgePolling(I2C_TypeDef *I2Cx,uint8_t I2C_Addr)
{
  vu16 SR1_Tmp;
  do
  {   
    I2C_GenerateSTART(I2Cx, ENABLE); 
    
    SR1_Tmp = I2C_ReadRegister(I2Cx, I2C_Register_SR1);
    
#ifdef AT24C01A

	I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Transmitter);
#else

	I2C_Send7bitAddress(I2Cx, 0, I2C_Direction_Transmitter);
#endif

  }while(!(I2C_ReadRegister(I2Cx, I2C_Register_SR1) & 0x0002));
  
  I2C_ClearFlag(I2Cx, I2C_FLAG_AF);
    
  I2C_GenerateSTOP(I2Cx, ENABLE);  /*Í£Ö¹Î»*/  
}


/*******************************************************************************
* Function Name  : I2C_Read
* Description    : 
* Input          : 
*                  
*                  
*                  
*                  
* Output         : None
* Return         : 
* Attention		 : None
*******************************************************************************/
uint8_t I2C_Read(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint16_t addr,uint8_t *buf,uint16_t num)
{
    if(num==0)
	return 1;
	
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
		

	I2C_AcknowledgeConfig(I2Cx, ENABLE);



    I2C_GenerateSTART(I2Cx, ENABLE);
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));/*EV5,Ö÷Ä£Ê½*/

#ifdef AT24C01A	

    I2C_Send7bitAddress(I2Cx,  I2C_Addr, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));


	I2C_SendData(I2Cx, (u8)((addr>>8)&0x00FF));
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_SendData(I2Cx, (u8)((addr)&0x00FF));
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));	
	/**/
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	
	/**/
	I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
		
#else	
	/*24C01*/
	I2C_Send7bitAddress(I2Cx, addr<<1, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
#endif
	
    while (num)
    {
		if(num==1)
		{
     		I2C_AcknowledgeConfig(I2Cx, DISABLE);	
    		I2C_GenerateSTOP(I2Cx, ENABLE);			
		}
	    
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)); 
	    *buf = I2C_ReceiveData(I2Cx);
	    buf++;
	    /* Decrement the read bytes counter */
	    num--;
    }
	/**/
	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	return 0;
}	

/*******************************************************************************
* Function Name  : I2C_WriteOneByte
* Description    : 
* Input          : - I2Cx:
*                  - I2C_Addr:
*                  - addr:
*                  - value:
* Output         : None
* Return         : 
* Attention		 : None
*******************************************************************************/
uint8_t I2C_WriteOneByte(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint16_t addr,uint8_t value)
{
    
  	I2C_GenerateSTART(I2Cx, ENABLE);
  	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));  

#ifdef AT24C01A
  	
  	I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Transmitter);
  	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
	I2C_SendData(I2Cx, (u8)((addr>>8)&0x00FF));
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_SendData(I2Cx, (u8)((addr)&0x00FF));
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));	

#else	
	I2C_Send7bitAddress(I2Cx, addr<<1, I2C_Direction_Transmitter);
 	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
#endif

	
  	I2C_SendData(I2Cx, value); 
  	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
  	
  	I2C_GenerateSTOP(I2Cx, ENABLE);
  
  	I2C_AcknowledgePolling(I2Cx,I2C_Addr);

	I2C_delay(1000);

	return 0;
}


/*******************************************************************************
* Function Name  : I2C_Write
* Description    : 
* Input          : - I2Cx:
*                  - I2C_Addr:
*                  - addr:·
*                  - *buf:
*                  - num:
* Output         : None
* Return         : 
* Attention		 : None
*******************************************************************************/
uint8_t I2C_Write(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint16_t addr,uint8_t *buf,uint16_t num)
{
	uint8_t err=0;
	
	while(num--)
	{
		if(I2C_WriteOneByte(I2Cx, I2C_Addr,addr++,*buf++))
		{
			err++;
		}
	}
	if(err)
		return 1;
	else 
		return 0;	
}


//--------------------------------------------------------------------------------------------------------
//

uint8_t EPPROM_Read(uint16_t addr,uint8_t *buf,uint16_t num){
		return I2C_Read(I2C1,ADDR_24LC02,0,buf,num );
}
uint8_t EPPROM_Write(uint16_t addr,uint8_t *buf,uint16_t num){
	return I2C_Write(I2C1,ADDR_24LC02,addr,buf,num);
}
uint16_t Addr=0;
uint8_t WriteBuffer[3000],ReadBuffer[3000];
void Test_ReadWriteEEp(void)
{
	int status=0;
  for(Addr=0; Addr<100; Addr++)
	WriteBuffer[Addr]=107;
	DEBUG("Start Debug EPPROM");
	I2C_Write(I2C1,ADDR_24LC02,0,WriteBuffer,sizeof(WriteBuffer) );
	status=1;
	I2C_Read(I2C1,ADDR_24LC02,0,ReadBuffer,sizeof(WriteBuffer) );
	status=1;
	if(  memcmp(WriteBuffer,ReadBuffer,sizeof(WriteBuffer)) == 0 ){
	status=1;
		DEBUG("Status %d",status);
		return;
	}
	else
	status=0;
	for(Addr=0; Addr<100; Addr++){
		status=(ReadBuffer[Addr]==107);
		DEBUG("Address %d - %d",status,ReadBuffer[Addr]);
	}
}







/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
