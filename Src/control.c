#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

extern int pitch;
extern 	int control;
void startControlTask(void const * argument) {
	char str_main[50];

	while(1){
		sprintf(str_main, "%3d	%3d\n", (int)control, (int)pitch);
		print(str_main);
		osDelay(50);

	}
}
