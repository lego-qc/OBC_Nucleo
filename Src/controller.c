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
#define maxControl 1000

extern int32_t numbers[4];
extern float yprDegree[3];
extern float angVel[3];

extern bool started;
extern bool stopped;
//extern int32_t ang_vel[3];
extern TIM_HandleTypeDef htim3;

float integrals[6];
float lastInputs[6];
float lastError[6];
float lastDerivs[6];
float lastDerivs2[6];
float lastOutputs[6];
float lastOutputs2[6];

float P_ang[] = {0, 15, 13};

float P[] = {0,   2.3, 2.2};
float I[] = {0, 0.3, 0.3};
float D[] = {0, 1.0, 1.0};

int32_t PID(int i, float input, float sp, float P, float I, float D){
	int32_t output = 0;
	float error = (sp - input);
	float deriv = 0;
	float filterd = 0;



	//calculate integrator only if it has effect, and limit its value
	if(I != 0){
	integrals[i] = integrals[i] + (error * 0.011);

		if(integrals[i] >= 120){
			integrals[i] = 120;
		}
		else if(integrals[i] <= -120){
			integrals[i] = -120;
		}
	}

	if(D != 0){
		//LPF for derivative term
		deriv = (error - lastError[i])/0.011 ; //todo: change to diff
		lastError[i] = error;

		filterd = 0.0675 * deriv + 0.1349 * lastDerivs[i] + 0.0675 * lastDerivs2[i] + 1.1430 * lastOutputs[i] - 0.4129 * lastOutputs2[i];
		//filterd = 0.1311 * deriv_rate + 0.2622 * lastU + 0.1311 * lastU2 + 0.7478 * lastFilterd - 0.2723 * lastFilterd2;
		lastDerivs[i] = deriv;
		lastDerivs2[i] = lastDerivs[i];
		lastOutputs[i] = filterd;
		lastOutputs2[i] = lastOutputs[i];
	}

	output = (int32_t)(error*(P) + integrals[i]*(I) + filterd*(D));

	return output;


}


void startControllerTask(TimerHandle_t t){
	float control_ang[3];
	int control[3];
	float locAngVel[3];
	float locYprDegree[3];

	char str_main[50];
	uint lastTicks = 0;
	int32_t diff = 0;
	uint32_t count = 0;
	int i = 0;

	for(i = 0; i < 6; i++){
		integrals[i] = 0;
		lastInputs[i] = 0;
		lastError[i] = 0;
		lastDerivs[i] = 0;
		lastDerivs2[i] = 0;
		lastOutputs[i] = 0;
		lastOutputs2[i] = 0;
	}


	while(1){

		if(stopped == true){
			motor1 =13000;
			motor2 =13000;
			motor3 =13000;
			motor4 =13000;
		}
		else if(started == true){
			int speed = 14240; //14240

			int motor1Loc = 0;
			int motor2Loc = 0;
			int motor3Loc = 0;
			int motor4Loc = 0;

			uint ticks = xTaskGetTickCount();







			diff = ticks - lastTicks;
			lastTicks = ticks;


			//copy to local variables
			for(i = 0; i < 3; i++){
				locAngVel[i] = angVel[i];
				locYprDegree[i] = yprDegree[i];
			}



			//perform angular controller on 5 times lower freq.
			if((count % 2) == 0){
				count = 0;

				for(i = 0; i < 3; i++){
					control_ang[i] = PID(i + 3, locYprDegree[i], numbers[i+1], P_ang[i], 0, 0);
				}

			}

			count++;

			for(i = 0; i < 3; i++){

				control[i] = PID(i, locAngVel[i], control_ang[i], P[i], I[i], D[i]);

				if(control[i] >= maxControl){
					control[i] = maxControl;
				}

				else if(control[i] <= -maxControl){
					control[i] = -maxControl;
				}
			}



			motor1Loc = (int)(speed + (control[2] * 1) + (control[1] * 1)) ;
			motor2Loc = (int)(speed + (control[2] * 1) - (control[1] * 1)) ;
			motor3Loc = (int)(speed - (control[2] * 1) - (control[1] * 1)) ;
			motor4Loc = (int)(speed - (control[2] * 1) + (control[1] * 1)) ;

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
		sprintf(str_main, "%6d %6d\n", (int)(round(yprDegree[2] * 100)), (int)round(yprDegree[1] *100) );
		print(str_main);
		osDelay(10);
	}
}
