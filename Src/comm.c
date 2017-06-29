#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdbool.h>
#include <string.h>


extern char Rxbuff[20];
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern int32_t number;


void startCommTask(void const * argument) {
	char msg1[] = "WIFI init succes!\n";
	char msg2[] = "WIFI init fail...\n";
	char last = 0;
	uint8_t e = 0;

	vTaskSuspendAll();


	if(ESPInit() == 0){
		HAL_UART_Transmit(&huart2, (uint8_t*)msg1, (uint16_t)strlen(msg1), (uint32_t)300);
	}
	else{
		HAL_UART_Transmit(&huart2, (uint8_t*)msg2, (uint16_t)strlen(msg2), (uint32_t)300);
	}


	while(1){
		if(HAL_UART_Receive(&huart3, (uint8_t*)&last, (uint16_t)1, (uint32_t)10) == HAL_OK){
			if(last == '\n'){
				break; //Connect message received

			}


		}
	}
	HAL_UART_Receive_DMA(&huart3, (uint8_t*)Rxbuff, (uint16_t)15);

	xTaskResumeAll();
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, GPIO_PIN_SET);

	while(1){
		int sign = 1;
		int received = false;
		char localCopy[15];

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		HAL_UART_DMAPause(&huart3);
		strncpy(localCopy, Rxbuff, 15);
		HAL_UART_DMAResume(&huart3);
		number = 0;


			for(e = 0; e < MSG_LEN; e++){
					if(localCopy[e] == '-'){
						sign = -1;
					}

					else if(localCopy[e] == ':'){
						received = true;
					}

					else if(received == true){
						number = number * 10 + (localCopy[e] - '0');
					}
			}

			number *= sign;


			HAL_UART_Receive_DMA(&huart3, (uint8_t*)Rxbuff, (uint16_t)15);





	}
}
