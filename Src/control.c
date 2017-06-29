#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

void startControlTask(void const * argument) {
	TaskStatus_t xTaskDetails;

	while(1){
		//vTaskGetTaskInfo(CommTaskHandle, &xTaskDetails, pdTRUE, 0);
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		osDelay(800);

	}
}
