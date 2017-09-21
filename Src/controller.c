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
	float deriv = 0;
	float lastErr = 0;
	char str_main[50];
	uint lastTicks = 0;
	int32_t diff = 0;


	while(1){

		if(stopped == true){
			motor1 =2400;
			motor2 =2400;
			motor3 =2400;
			motor4 =2400;
		}
		else if(started == true){
			float error = 0;
			int speed = 2750;

			float locPitch = pitch;
			int motor1Loc = 0;
			int motor2Loc = 0;
			int motor3Loc = 0;
			int motor4Loc = 0;

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

			deriv = (locPitch - lastErr)/0.011 ; //thesis alapján
			lastErr = locPitch;

			control = (int)(error*((numbers[1]) / 10.0) + integral*((numbers[2]) / 10.0) - deriv*((numbers[3]) / 10.0));


			if(control >= 400){
				control = 400;
			}

			else if(control <= -400){
				control = -400;
			}

			motor1Loc = speed + control;
			motor2Loc = speed + control;
			motor3Loc = speed - control;
			motor4Loc = speed - control;

			if(motor1Loc >= 2700){
				motor1 = motor1Loc;
			}
			else{
				motor1 = 2700;
			}

			if(motor2Loc >= 2700){
				motor2 = motor2Loc;
			}
			else{
				motor2 = 2700;
			}

			if(motor3Loc >= 2700){
				motor3 = motor3Loc;
			}
			else{
				motor3 = 2700;
			}

			if(motor4Loc >= 2700){
				motor4 = motor4Loc;
			}
			else{
				motor4 = 2700;
			}




		}
		sprintf(str_main, "%3d %3d\n", (int)(pitch*10), (int)(control*10));
		print(str_main);
		osDelay(10);
	}
}
