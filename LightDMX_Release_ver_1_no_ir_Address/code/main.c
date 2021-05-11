

#include "Main_Task.h"
#include "24C02.h"

void ProjectInit(){
	
}
 
int main(void)
{
	MainTask_Init();
	SysTick_Config(SystemCoreClock / 1000);
	vTaskStartScheduler();
	return 0;
}


	
