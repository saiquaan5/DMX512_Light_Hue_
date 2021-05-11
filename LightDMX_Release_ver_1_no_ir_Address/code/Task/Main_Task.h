/*
 * Main_Task.h
 *
 *  Created on: Nov 1, 2016
 *      Author: Phap.Ha
 */
 

 #include "FreeRTOS.h"
 #include "task.h"
 #include "queue.h"
 #include "stm32f10x.h"
 #include "stm32f10x_gpio.h"
 #include "timers.h"
 #include "dmx512_rec.h"
 #include <stdarg.h>
 #include <stdio.h >
 #include <string.h>
 #include "LightDMX.h"
 #include "24C02.h"
 #include "PWM.h"

extern TimerHandle_t xTimers;
 
	void MainTask_Init(void);
	void DMX_Timeout( void *pvParameters );
