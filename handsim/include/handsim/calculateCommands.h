// Header file to read in commands

#include <haptix/comm/haptix.h>
#include <stdbool.h>
#include <math.h>
#include "EMGStruct.h"

// calculateCommands.h
struct EMGData;
void calculateCommands(hxRobotInfo *robotInfo, hxCommand *cmd, hxSensor *sensor, struct EMGData *emg, bool usingEMG, int counter);