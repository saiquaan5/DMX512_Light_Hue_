/*
 * Main_Task.c
 *
 *  Created on: Nov 1, 2016
 *      Author: Phap.Ha
 */
 
 #include "Main_Task.h"
 #include "serial_tracer.h"
 #include "PWM.h"
 #define Main_TASK_STACK_SIZE 512
 #define Main_TASK_PRIORITY 	3

QueueHandle_t  xDMX_Queue;


static void vMainTask( void *pvParameters )
{
	DMX_Command_Queue ReceiveData=DEBUGRED_COMMAND_QUEUE;

 uint8_t res;
 res=0;
	
 I2C_Configuration();
	#ifdef DEBUG_EPPROM
//	Test_ReadWriteEEp();
	#endif
	
	pwm_init();
//	dmxR(0);
//	dmxG(0);
//	dmxB(0);
 u8_LightDMX_Init(dmx512_init,EPPROM_Read,EPPROM_Write,pwm_set_output);
//	xQueueSend( xDMX_Queue, (void*)&ReceiveData,0);
 for(;;)
 {
	 u8_LightDMX_Run();

 }
}
 
 
 void MainTask_Init(void)
 {	

	Usart_Init(9600);
	printf2("Init MainTask");
	 
	xDMX_Queue = xQueueCreate( 5, sizeof(DMX_Command_Queue));
	if( xDMX_Queue == NULL )
	{
		printf2("Create xDMX_Queue Fail");
			/* Queue was not created and must not be used. */
	}
//	xTimerOuts=xTimerCreate("Timer",xTicksToWait,pdFALSE,0,DMX_Timeout);
	xTaskCreate( vMainTask, "Main", Main_TASK_STACK_SIZE, NULL, Main_TASK_PRIORITY, NULL );
 }
 