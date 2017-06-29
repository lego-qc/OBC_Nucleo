#ifndef SHAREDRESOURCES_H_
#define SHAREDRESOURCES_H_

#include "types.h"

void initializeResources();

state getCurrentState();
state getDesiredState();

void setCurrentState(state s);
void setDesiredState(state s);

#endif
