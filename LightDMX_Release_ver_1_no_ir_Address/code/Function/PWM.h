
#ifndef _PWM_H
#define _PWM_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "serial_tracer.h"
#define dmxR(x)  TIM3->CCR1=x
#define dmxB(x)  TIM3->CCR2=x
#define dmxG(x)  TIM3->CCR3=x

void pwm_init(void);
void pwm_out_init(TIM_TypeDef *timx);
void pwm_set_output(volatile uint8_t *data);

#endif