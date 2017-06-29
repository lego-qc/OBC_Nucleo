#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim3;

void startPwmTask(void const * argument) {
	while(1){
		htim3.Instance->CCR1 =1000;
		htim3.Instance->CCR2 =1500;
		htim3.Instance->CCR3 =1000;
		htim3.Instance->CCR4 =1500;
		osDelay(1000);

	}
}
