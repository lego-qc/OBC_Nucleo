#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdbool.h>
#include <string.h>


extern char Rxbuff[MSG_LEN];
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern int32_t numbers[4];
extern bool started;
extern bool stopped;


void startCommTask(void const * argument) {
	char msg1[] = "WIFI init succes!\n";
	char msg2[] = "WIFI init fail...\n";
	char start[] = "START_______________\0";
	char stop[] = "STOP________________\0";
	char textMsg[MSG_LEN];
	char last = 0;

	uint8_t e = 0;

	osDelay(500);
	vTaskSuspendAll();


	if(ESPInit() == 0){
	HAL_UART_Transmit(&huart2, (uint8_t*)msg1, (uint16_t)strlen(msg1), (uint32_t)300);
	}
	else{
		HAL_UART_Transmit(&huart2, (uint8_t*)msg2, (uint16_t)strlen(msg2), (uint32_t)300);
	}


	while(1){
		if(HAL_UART_Receive(&huart3, (uint8_t*)&last, (uint16_t)1, (uint32_t)10) == HAL_OK){
			HAL_UART_Transmit(&huart2, &last,1,200);
			if(last == '\n'){
				break; //Connect message received

			}


		}
	}
	HAL_UART_Receive_DMA(&huart3, (uint8_t*)Rxbuff, (uint16_t)MSG_LEN);

	xTaskResumeAll();
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, GPIO_PIN_SET);

	while(1){
		int sign = 1;
		int received = false;
		char localCopy[MSG_LEN];
		int i = 0;
		int l = 0;
		int numbersCount = 0;


		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		strncpy(localCopy, Rxbuff, MSG_LEN); //old: 15 (11 + 4)
		HAL_UART_Receive_DMA(&huart3, (uint8_t*)Rxbuff, (uint16_t)MSG_LEN);
		for(l = 0; l < 4; l++){
			numbers[l] = 0;
		}

		for(l = 0; l < MSG_LEN; l++){
			textMsg[l] = 0;
		}



		for(e = 0; e < MSG_LEN; e++){
				if(localCopy[e] == '-'){
					sign = -1;
				}

				else if(localCopy[e] == ':'){
					received = true;
				}

				else if(received == true){
					if((localCopy[e] >= '0') && (localCopy[e] <= '9')){
						numbers[numbersCount] = numbers[numbersCount] * 10 + (localCopy[e] - '0');
					}

					else if(localCopy[e] == '%'){
						if(numbersCount < 4){
							numbers[numbersCount] *= sign;
							numbersCount++;
							sign = 1;
						}
					}

					else{
						textMsg[i] = localCopy[e];
						i++;
					}
				}
		}

		textMsg[i] = '\0';
		print(textMsg);

		if(strcmp(textMsg, start) == 0){
			started = true;
		}

		else if(strcmp(textMsg, stop) == 0){
			stopped = true;
		}



	}
}
