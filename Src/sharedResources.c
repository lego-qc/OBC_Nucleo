#include "types.h"
#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t currentStateHandler;
static SemaphoreHandle_t desiredStateHandler;

static state currentState;
static state desiredState;

void initializeResources() {
	currentStateHandler = xSemaphoreCreateBinary();
	desiredStateHandler = xSemaphoreCreateBinary();
}

state getCurrentState() {
	xSemaphoreTake(currentStateHandler, portMAX_DELAY);
	state copy = currentState;
	xSemaphoreGive(currentStateHandler);
	return copy;
}

state getDesiredState() {
	xSemaphoreTake(desiredStateHandler, portMAX_DELAY);
	state copy = desiredState;
	xSemaphoreGive(desiredStateHandler);
	return copy;
}

void setCurrentState(state s) {
	xSemaphoreTake(currentStateHandler, portMAX_DELAY);
	currentState = s;
	xSemaphoreGive(currentStateHandler);
}

void setDesiredState(state s) {
	xSemaphoreTake(desiredStateHandler, portMAX_DELAY);
	desiredState = s;
	xSemaphoreGive(desiredStateHandler);
}
