#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdbool.h>

extern int32_t number;
extern int pitch;
extern bool started;
extern bool stopped;
extern TIM_HandleTypeDef htim3;

void startControllerTask(TimerHandle_t t){
	char str_main[50];


	while(1){

		if(stopped == true){
			htim3.Instance->CCR1 =80;
			htim3.Instance->CCR2 =80;
			htim3.Instance->CCR3 =80;
			htim3.Instance->CCR4 =80;
		}
		else if(started == true){
			int error = 0;
			float control = 0;
			int speed = 100;

			error = number - pitch;
			control = error * 0.4;

			if(control <= 30){
				htim3.Instance->CCR1 = speed + control;
				htim3.Instance->CCR2 = speed + control;
				htim3.Instance->CCR3 = speed - control;
				htim3.Instance->CCR4 = speed - control;
			}
			else{
				htim3.Instance->CCR1 =80;
				htim3.Instance->CCR2 =80;
				htim3.Instance->CCR3 =80;
				htim3.Instance->CCR4 =80;
			}

			sprintf(str_main, "Pitch: %3d Req: %3d\n", (int)pitch, (int)number);
			print(str_main);




		}

		osDelay(10);
	}
}
