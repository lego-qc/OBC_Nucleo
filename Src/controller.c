#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

extern int32_t number;
extern int pitch;


void startControllerTask(TimerHandle_t t){
	char str_main[50];

	while(1){
		sprintf(str_main, "Pitch: %3d Req: %3d\n", (int)pitch, (int)number);
		print(str_main);

		osDelay(50);
	}
}
