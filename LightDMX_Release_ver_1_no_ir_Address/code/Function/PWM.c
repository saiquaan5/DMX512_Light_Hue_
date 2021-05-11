#include "PWM.h"

void pwm_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
//		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
//	GPIO_SetBits(GPIOB,GPIO_Pin_0);
//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, DISABLE);
	pwm_out_init(TIM3);
}

void pwm_out_init(TIM_TypeDef *timx)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t prescalerValue = 0;

	prescalerValue = (uint16_t) (SystemCoreClock / 120000 ) - 1;
/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 255 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(timx, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

	TIM_OC1Init(timx, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(timx, TIM_OCPreload_Enable);

	TIM_OC2Init(timx, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(timx, TIM_OCPreload_Enable);


	TIM_OC3Init(timx, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(timx, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(timx, ENABLE);
	TIM_Cmd(timx, ENABLE);
}

void pwm_set_output(volatile uint8_t *data)
{
	printf2("PWM %d %d %d \r\n",data[0],data[1],data[2]);
	dmxR(data[0]);
	dmxG(data[1]);
	dmxB(data[2]);	
}