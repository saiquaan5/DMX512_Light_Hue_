/**
  ******************************************************************************
  * @file    RTC/Calendar/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "dmx512_rec.h"
#include "LightDMX.h"
#include "timers.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup RTC_Calendar
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t TimeDisplay;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/
static volatile int dmx_error = 1;
static volatile int byte_count = 0;
static volatile int start_flag = 0;
static enum packet_type packet_type;


static volatile uint8_t dmx_status=0;

void DMX_WaitBreakTime(uint8_t);
void DMX_WaitForStart(uint8_t);
void DMX_ReadData(uint8_t);
static void (*DMX_STATUS)(uint8_t )=DMX_WaitBreakTime;

void DMX_WaitBreakTime(uint8_t rx)
{
	if(start_flag==1){
		BaseType_t xHigherPriorityTaskWoken;
	DMX_Command_Queue ubuff=OUTPUT_COMMAND_QUEUE;
	/* We have not woken a task at the start of the ISR. */
	xHigherPriorityTaskWoken = pdFALSE;
	dmx_error = 0;
	byte_count = 0;
	start_flag = 0;
	
	xQueueSendFromISR( xDMX_Queue, (void*)&ubuff, &xHigherPriorityTaskWoken);
		
	switch (rx) {
		case 0x0:
			packet_type = DATA_PACKET;
			break;
		case 0x17:
			packet_type = TEST_PACKET;
			break;
		default:
			dmx_error = 1;
			DMX_STATUS=DMX_WaitBreakTime;
			TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
			return;
		}
	DMX_STATUS=DMX_ReadData;
	}
}

void DMX_WaitForStart(uint8_t rx){
	
		
}

void DMX_ReadData(uint8_t rx){
	if(byte_count>=lightDMX_data.channel){
		lightDMX_data.value[byte_count-lightDMX_data.channel]=rx;
	}
	byte_count++;
	if(byte_count >=lightDMX_data.channel+DMX_VALUE_SIZE){
		DMX_STATUS=DMX_WaitBreakTime;
	}
}


void TIM1_CC_IRQHandler(void)
{
	uint32_t break_time, mab_time;
//  Time_out_pulse=0;
	TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);

	break_time = TIM_GetCapture2(TIM1);
	mab_time = TIM_GetCapture1(TIM1) - break_time;

	/* use some tolerance for times */
	if (break_time > 80 && break_time < 10e3 && mab_time > 8) {
		start_flag = 1;
		TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
	}

}

void USART2_IRQHandler(void)
{
	uint8_t rx_byte;
	int16_t fe_flag;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);

	fe_flag = USART_GetFlagStatus(USART2, USART_FLAG_FE);
	rx_byte = USART_ReceiveData(USART2); /* also clears FE flag */

	if (fe_flag) {
		TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
		return;
	}
	DMX_STATUS(rx_byte);
}
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles RTC global interrupt request.
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
  if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
  {
    /* Clear the RTC Second interrupt */
    RTC_ClearITPendingBit(RTC_IT_SEC);

    /* Toggle LED1 */

    /* Enable time update */
		
		
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
    
  }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
void EXTI15_10_IRQHandler(void)
{
//	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
//	{
//		switch(ReadBKPdata16(DriverMode))
//		{
//			case ModeEncoder:
//			{
//				if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12))
//				{
//					//i32_AUp();
//					if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_13))
//						{
//							// encoder +
//							i32_SetEncoderZUp();
//						}
//							else
//						{
//							i32_SetEncoderZDown();
//						}
//				}
//				else
//				{
//					if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_13))
//						{
//							i32_SetEncoderZDown();
//						}
//							else
//						{
//							i32_SetEncoderZUp();
//						}
//				}
//			}
//			break;
//			
//			case ModeStepDir:
//			{
//				//Pin 12 = DIR
//			}
//			break;
//			
//			case ModeCW_CCW:
//			{
//				if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12))
//				{
//					i32_SetEncoderZUp();
//				}
//			}
//			break;
//		}
//		
//		EXTI->PR = EXTI_Line12;
//	}
//	
//	if(EXTI_GetITStatus(EXTI_Line13) != RESET)
//	{
//		switch(ReadBKPdata16(DriverMode))
//		{
//			case ModeEncoder:
//			{
//				if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_13))
//				{
//					if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12))
//						{
//							i32_SetEncoderZDown();
//						}
//							else
//						{
//							i32_SetEncoderZUp();
//						}
//				}
//				else
//				{
//					if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12))
//						{
//							i32_SetEncoderZUp();
//						}
//							else
//						{
//							i32_SetEncoderZDown();
//						}
//				}
//			}
//			break;
//			case ModeStepDir:
//			{
//				if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_13))
//				{
//					if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12))
//						{
//							i32_SetEncoderZUp();
//						}
//						else
//						{
//							i32_SetEncoderZDown();
//						}
//				}
//			}
//			break;
//			case ModeCW_CCW:
//			{
//				if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12))
//				{
//					i32_SetEncoderZDown();
//				}
//			}
//			break;
//		}
//			
//		
//		EXTI->PR = EXTI_Line13;
//	}
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
