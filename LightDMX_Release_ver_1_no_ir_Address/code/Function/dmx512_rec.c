
#include "dmx512_rec.h"

struct dmx512_config {
	TIM_TypeDef *tim;
	USART_TypeDef *usart;
	int start_addr;
};

static struct dmx512_config dmx512_config;

void NVIC_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
	

void dmx512_init(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

/* USART_RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

/* USART_TX - FIX OF HW BUG */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

/* Timer_CH1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

/* Init DMX Module*/
	dmx512_rec_init(TIM1, USART2);
	NVIC_init();
	dmx512_rec_enable(1);
}


/*
 * configures timer for detecting BREAK and MAB
 * UART for dmx512 data
 */
void dmx512_rec_init(TIM_TypeDef *timx, USART_TypeDef *usartx){
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	USART_InitTypeDef USART_InitStructure;

	dmx512_config.tim = timx;
	dmx512_config.usart = usartx;
	dmx512_config.start_addr = 0;

	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/1e6 - 1; /* timer tics are in us */
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(timx, &TIM_TimeBaseStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_PWMIConfig(timx, &TIM_ICInitStructure);

	TIM_SelectInputTrigger(timx, TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(timx, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(timx, TIM_MasterSlaveMode_Enable);

	TIM_Cmd(timx, ENABLE);

	USART_InitStructure.USART_BaudRate = 250000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;

	USART_Init(usartx, &USART_InitStructure);
	USART_Cmd(usartx, ENABLE);
}



void dmx512_rec_enable(int on){
	if (on) {
		USART_ITConfig(dmx512_config.usart, USART_IT_RXNE, ENABLE);
		TIM_ITConfig(dmx512_config.tim, TIM_IT_CC1, ENABLE);
	} else {
		USART_ITConfig(dmx512_config.usart, USART_IT_RXNE, DISABLE);
		TIM_ITConfig(dmx512_config.tim, TIM_IT_CC1, DISABLE);
	}
}
