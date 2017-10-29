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

UART_HandleTypeDef huart2;

void startSensorTask(void const * argument) {
	uint8_t fifoBuffer[64]; // FIFO storage buffer
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorFloat gravity;
	float ypr[3];
	float prev[3];
	int i = 0;

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
			MPUgetFIFOBytes(fifoBuffer, packetSize);
			fifoCount -= packetSize;
			MPUdmpGetQuaternion(&q, fifoBuffer);
			//MPUdmpGetGyro32(ang_vel, fifoBuffer);

			MPUdmpGetGravityVect(&gravity, &q);
			MPUdmpGetYawPitchRoll(ypr, &q, &gravity);


			for(i = 0; i < 3; i++){
				yprDegree[i] = ((ypr[i] * 180.0)/M_PI);
				angVel[i] = (yprDegree[i] - prev[i])*100;
				prev[i] = yprDegree[i];
			}
		}

	}

}
