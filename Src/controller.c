#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdbool.h>

#define motor1 (htim3.Instance->CCR1)
#define motor2 (htim3.Instance->CCR3)
#define motor3 (htim3.Instance->CCR2)
#define motor4 (htim3.Instance->CCR4)

extern int32_t numbers[4];
extern float pitch;
extern bool started;
extern bool stopped;
extern TIM_HandleTypeDef htim3;
extern int control;

void startControllerTask(TimerHandle_t t){
	float integral = 0;
	float lastErr = 0;
	char str_main[50];
	uint lastTicks = 0;
	int32_t diff = 0;


	while(1){

		if(stopped == true){
			motor1 =800;
			motor2 =800;
			motor3 =800;
			motor4 =800;
		}
		else if(started == true){
			float error = 0;
			int speed = 910;
			float deriv = 0;
			float locPitch = pitch;
			uint ticks = xTaskGetTickCount();
			diff = ticks - lastTicks;
			lastTicks = ticks;

			error = numbers[0] - locPitch;
			integral = integral + (error * 0.011);


			if(integral >= 120){
				integral = 120;
			}
			else if(integral <= -120){
				integral = -120;
			}

			deriv = (error - lastErr)/0.011 ;
			lastErr = error;

			control = (int)(error*(numbers[1] / 10.0) + integral*(numbers[2] / 10.0) + deriv*(numbers[3] / 10.0));


			if(control >= 200){
				control = 200;
			}

			else if(control <= -200){
				control = -200;
			}
			if(control > 0){
				motor1 = speed + control;
				motor2 = speed + control;
				motor3 = speed;
				motor4 = speed;
			}

			else{
				motor1 = speed;
				motor2 = speed;
				motor3 = speed - control;
				motor4 = speed - control;
			}









		}
		sprintf(str_main, "%3d	%3d\n", (int)numbers[0], (int)pitch);
		print(str_main);
		osDelay(10);
	}
}
