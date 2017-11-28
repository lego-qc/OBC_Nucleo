#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "iic.h"
#include <math.h>

volatile uint8_t mpuInt;
volatile uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
volatile uint16_t fifoCount;     // count of all bytes currently in FIFO

extern float yprDegree[3];
extern float angVel[3];
extern int32_t ang_vel[3];
extern int32_t ang_vel2[3];
extern float euler[3];

UART_HandleTypeDef huart2;

void startSensorTask(void const * argument) {
	uint8_t fifoBuffer[64]; // FIFO storage buffer
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorFloat gravity;
	float ypr[3];
	float prev[3];
	int i = 0;
	uint diff = 0;
	uint lastTicks = 0;
	float freq = 0;

	float filterd[3];
	float lastInputs[3];
	float lastInputs2[3];
	float lastOutputs[3];
	float lastOutputs2[3];


	for(i = 0; i < 3; i++){
		lastInputs[i] = 0;
		lastInputs2[i] = 0;
		lastOutputs[i] = 0;
		lastOutputs2[i] = 0;
		prev[i] = 0;
	}



//	osDelay(5000);
	vTaskSuspendAll();

	MPU6050(0xD0);
	MPUinitialize();

	if (MPUtestConnection() == SUCCESS) {
		print("Gyro started\n");
	}
	else{
		print("Gyro failed\n");
	}

	MPUdmpInitialize();
	MPUsetDMPEnabled(true);

	xTaskResumeAll();

	mpuIntStatus = MPUgetIntStatus();
	//packetSize = MPUdmpGetFIFOPacketSize(); //42 mindig




	while (1) {
		/* Infinite loop */

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		mpuIntStatus = MPUgetIntStatus();
		fifoCount = MPUgetFIFOCount();

		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			char ovf[] = "FIFO overflow!\n";
			MPUsetFIFOEnabled(false);
			MPUresetFIFO();
			MPUsetFIFOEnabled(true);

			//MPUreset();

			mpuIntStatus = MPUgetIntStatus();
			fifoCount = MPUgetFIFOCount();


			HAL_UART_Transmit(&huart2, (uint8_t*)ovf, (uint16_t)strlen(ovf), (uint32_t)300);

		}

		else if ((mpuIntStatus & 0x01) && (fifoCount >= packetSize)) {
			float angDiff = 0;
			uint ticks = xTaskGetTickCount();

			diff = ticks - lastTicks;
			lastTicks = ticks;
			freq = (1.0 / diff) * 1000;


			MPUgetFIFOBytes(fifoBuffer, packetSize);
			fifoCount -= packetSize;
			MPUdmpGetQuaternion(&q, fifoBuffer);
			MPUdmpGetGyro32(ang_vel, fifoBuffer);

			MPUdmpGetGravityVect(&gravity, &q);
			MPUdmpGetYawPitchRoll(ypr, &q, &gravity);
			MPUdmpGetEuler(euler, &q);

//			ang_vel[2] = ang_vel[2] * -0.00007;
			ang_vel[0] = ang_vel[0] * -0.0000467;
			ang_vel[1] = ang_vel[1] * 0.0000252;

			for(i = 0; i < 3; i++){
				yprDegree[i] = ((ypr[i] * 180.0)/M_PI);
			}

			for(i = 0; i < 2; i++){

				//angVel[i] = (yprDegree[i] - prev[i])*freq;
				//prev[i] = yprDegree[i];
				//ang_vel2[i] = angVel[i];

				//ang_Vel and ypr has different order
				filterd[i] = 0.23738 * ang_vel[i] + 0.23738 * lastInputs[i] + 0.52525 * lastOutputs[i];

				lastInputs[i] = ang_vel[i];
				lastInputs2[i] = lastInputs[i];
				lastOutputs[i] = filterd[i];
				lastOutputs2[i] = lastOutputs[i];

			}


			//compensate yaw angle disruption
			angDiff = yprDegree[0] - prev[0];

			if(angDiff > 180){
				angDiff = angDiff - 360;
			}
			else if(angDiff < -180){
				angDiff = angDiff + 360;
			}

			angVel[0] = (angDiff)*freq;
			prev[0] = yprDegree[0];

			angVel[2] = filterd[0];
			angVel[1] = filterd[1];

		}

	}

}
