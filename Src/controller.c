#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define motor1 (htim3.Instance->CCR1)
#define motor2 (htim3.Instance->CCR3)
#define motor3 (htim3.Instance->CCR2)
#define motor4 (htim3.Instance->CCR4)

#define minPWM 14000
#define maxControl 5000
#define linSectionLength 1500 //1350
#define dT 0.011

#define base_spd 14500

extern int32_t numbers[4];
extern float yprDegree[3];
extern float angVel[3];
extern int32_t ang_vel[3];
extern int32_t ang_vel2[3];
extern float euler[3];

extern bool started;
extern bool stopped;
extern TIM_HandleTypeDef htim3;

float integrals[6];
float lastInputs[6];
float lastError[6];
float lastErrors[6];
float lastErrors2[6];
float lastOutputs[6];
float lastOutputs2[6];

float tmp[6];

float P_ang[] = {0, 4, 5.5}; //0 3.5 3

float P[] = {40,   2.5, 2.8};
float I[] = {0.9, 0.8, 1.8};
float D[] = {0.5, 0.4, 0.4};
float maxAccel[] = {40, 500, 500};
float offset[] = {0, 0.33, -4.71};

int32_t PID(int i, float input, float sp, float P, float I, float D){
	int32_t output = 0;
	float error = (sp - input);
	float deriv = 0;
	float filterd = 0;



	//calculate integrator only if it has effect, and limit its value
	if(I != 0){
	integrals[i] = integrals[i] + (error * dT);

		if(integrals[i] >= 500){
			integrals[i] = 500;
		}
		else if(integrals[i] <= -500){
			integrals[i] = -500;
		}
	}

	if(D != 0){
		//LPF for derivative term
		filterd = 0.0675 * error + 0.1349 * lastErrors[i] + 0.0675 * lastErrors2[i] + 1.1430 * lastOutputs[i] - 0.4129 * lastOutputs2[i];

		lastErrors[i] = error;
		lastErrors2[i] = lastErrors[i];
		lastOutputs[i] = filterd;
		lastOutputs2[i] = lastOutputs[i];


		deriv = (filterd - lastError[i])/dT ; //todo: change to diff
		lastError[i] = filterd;

	}
	//tmp[i] = filterd;
	output = (int32_t)(error*(P) + integrals[i]*(I) + deriv*(D));

	return output;


}


void startControllerTask(TimerHandle_t t){
	float control_ang[3];
	float lastControl_ang[3];
	int control[3];
	float locAngVel[3];
	float locYprDegree[3];

	char str_main[100];
	uint lastTicks = 0;
	int32_t diff = 0;
	uint32_t count = 0;
	int i = 0;

	for(i = 0; i < 6; i++){
		integrals[i] = 0;
		lastInputs[i] = 0;
		lastError[i] = 0;
		lastErrors[i] = 0;
		lastErrors2[i] = 0;
		lastOutputs[i] = 0;
		lastOutputs2[i] = 0;
	}

	for(i = 0; i < 3; i++){
		lastControl_ang[i] = 0;
	}



	while(1){

		if(stopped == true){
			motor1 =13000;
			motor2 =13000;
			motor3 =13000;
			motor4 =13000;
		}
		else if(started == true){
			int speed = 0.0;

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
				locYprDegree[i] = yprDegree[i] - offset[i];
			}



			//perform angular controller on 2 times lower freq.
			if((count % 2) == 0){
				count = 0;


				//calculate pitch and roll P rate output
				for(i = 0; i < 3; i++){
					float delta_ang_vel = 0.0;

					if(i != 0){
						float error = numbers[i+1] - locYprDegree[i];
						float lin_section = linSectionLength / (P_ang[i] * P_ang[i]);


						if(error > lin_section){
							control_ang[i] = sqrt(2.0 * linSectionLength * (error - (lin_section / 2.0)));
						}
						else if(error < (-lin_section)){
							control_ang[i] = -sqrt(2.0 * linSectionLength * (-error - (lin_section / 2.0)));
						}
						else{
							control_ang[i] = P_ang[i] * error;
						}


						//tmp[i] = control_ang[i];
					}
					else{
						//Yaw rate controller input is directly defined by the user
						control_ang[0] = numbers[1];
					}


					//input limitation to the rate controller
					delta_ang_vel = maxAccel[i] * (2*dT); //period time is 2*dT

					if(control_ang[i] < (lastControl_ang[i] - delta_ang_vel)){
						control_ang[i] =  lastControl_ang[i] - delta_ang_vel;
					}

					else if(control_ang[i] > (lastControl_ang[i] + delta_ang_vel)){
						control_ang[i] = lastControl_ang[i] + delta_ang_vel;
					}

					lastControl_ang[i] = control_ang[i];


				}

			}

			count++;

			speed = base_spd + (numbers[0] * 100);


			//angular PID controller with saturation
			for(i = 0; i < 3; i++){
				float integral = 0;

				//use integrator only after the take off
				if(speed >= (base_spd + 100)){ //~2000 enough to take off
					integral = I[i];
				}

				control[i] = PID(i, locAngVel[i], control_ang[i], P[i], integral, D[i]);

				if(control[i] >= maxControl){
					control[i] = maxControl;
				}

				else if(control[i] <= -maxControl){
					control[i] = -maxControl;
				}
			}



			motor1Loc = (int)(speed + (control[2] ) + (control[1] ) + ( control[0]));
			motor2Loc = (int)(speed + (control[2] ) - (control[1] ) - ( control[0]));
			motor3Loc = (int)(speed - (control[2] ) - (control[1] ) + ( control[0]));
			motor4Loc = (int)(speed - (control[2] ) + (control[1] ) - ( control[0]));

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

			tmp[0] = motor1Loc;
			tmp[1] = motor3Loc;



		}

		sprintf(str_main, "%6d %6d %6d %6d %6d %6d %6d %6d %6d\n", (int)(round(control[0] * 100)), (int)(round(control[1] * 100)), (int)(round(control[2] * 100)), (int)(round(angVel[0] * 100)), (int)(round(angVel[1] * 100)),(int)(round(angVel[2] * 100)), (int)(round(locYprDegree[0] * 100)),(int)(round(locYprDegree[1] * 100)),(int)(round(locYprDegree[2] * 100)));
		//sprintf(str_main, "%6d %6d %6d %6d %6d %6d %6d %6d %6d\n", (int)(round(tmp[0])), (int)(round(tmp[1])), (int)(round(control[2] * 100)), (int)(round(angVel[0] * 100)), (int)(round(angVel[1] * 100)),(int)(round(angVel[2] * 100)), (int)(round(yprDegree[0] * 100)),(int)(round(yprDegree[1] * 100)),(int)(round(yprDegree[2] * 100)));
		print(str_main);
		osDelay(10);
	}
}
