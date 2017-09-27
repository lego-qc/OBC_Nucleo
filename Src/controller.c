#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define motor1 (htim3.Instance->CCR1)
#define motor2 (htim3.Instance->CCR3)
#define motor3 (htim3.Instance->CCR2)
#define motor4 (htim3.Instance->CCR4)

#define minPWM 13800

extern int32_t numbers[4];
extern float pitch;
extern float pitchVel;
extern bool started;
extern bool stopped;
//extern int32_t ang_vel[3];
extern TIM_HandleTypeDef htim3;

void startControllerTask(TimerHandle_t t){
	float error_ang = 0;
	float error_rate = 0;
	float integral_rate = 0;
	float deriv_rate = 0;
	float lastErr_rate = 0;
	float control_ang = 0;
	int control = 0;
	double lastPitch = 0;
	double locAngSpd = 0;
	float filterd = 0;
	float lastFilterd = 0;
	float lastFilterd2 = 0;
	float lastFilterd3 = 0;
	float lastU = 0;
	float lastU2 = 0;



	char str_main[50];
	uint lastTicks = 0;
	int32_t diff = 0;
	uint32_t count = 0;


	while(1){

		if(stopped == true){
			motor1 =13000;
			motor2 =13000;
			motor3 =13000;
			motor4 =13000;
		}
		else if(started == true){
			int speed = 14240; //2750
			double locPitch = pitch;

			int motor1Loc = 0;
			int motor2Loc = 0;
			int motor3Loc = 0;
			int motor4Loc = 0;

			uint ticks = xTaskGetTickCount();
			float P_ang = numbers[1]; //14
			float P_rate = numbers[2]/ 10.0; //1
			float I_rate = 0.2; //0.2
			float D_rate = numbers[2] /10.0; //0.4


			diff = ticks - lastTicks;
			lastTicks = ticks;


			//calculate angular speed
			locAngSpd = pitchVel;



			//perform angular controller on 5 times lower freq.
			if((count % 5) == 0){
				count = 0;

				error_ang = numbers[0] - locPitch;
				control_ang = P_ang * error_ang;

			}

			count++;


			//angular rate controller. input: control_ang (desired angular velocity), output: control
			error_rate = control_ang - locAngSpd;

			//LPF for derivative term


			deriv_rate = (error_rate - lastErr_rate)/0.011 ; //todo: change to diff
			lastErr_rate = error_rate;

			filterd = 0.4 * deriv_rate + 0.3 * lastU + 0.3 * lastU2
			//filterd = 0.1311 * deriv_rate + 0.2622 * lastU + 0.1311 * lastU2 + 0.7478 * lastFilterd - 0.2723 * lastFilterd2;
			lastFilterd = filterd;
			lastFilterd2 = lastFilterd;
			//lastFilterd3 = lastFilterd2;
			lastU = deriv_rate;
			lastU2 = lastU;

			control = (int)(error_rate*(P_rate) + integral_rate*(I_rate) + filterd*(D_rate));


			//calculate integrator only if it has effect, and limit its value
			if(I_rate != 0){
			integral_rate = integral_rate + (error_ang * 0.011);

				if(integral_rate >= 120){
					integral_rate = 120;
				}
				else if(integral_rate <= -120){
					integral_rate = -120;
				}
			}


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

			if(motor1Loc >= minPWM){
				motor1 = motor1Loc;
			}
			else{
				motor1 = minPWM;
			}

			if(motor2Loc >= minPWM){
				motor2 = motor2Loc;
			}
			else{
				motor2 = minPWM;
			}

			if(motor3Loc >= minPWM){
				motor3 = motor3Loc;
			}
			else{
				motor3 = minPWM;
			}

			if(motor4Loc >= minPWM){
				motor4 = motor4Loc;
			}
			else{
				motor4 = minPWM;
			}





		}
		sprintf(str_main, "%6d %6d\n", (int)(round(filterd * 100)), (int)round(deriv_rate *100) );
		print(str_main);
		osDelay(10);
	}
}
