// Header file for calculateCommands.c
//
// Mikey Fernandez 04/11/2021

#ifndef CALCULATE_COMMANDS_H
#define CALCULATE_COMMANDS_H

#include <haptix/comm/haptix.h>
#include <stdbool.h>
#include <math.h>
#include "EMGStruct.h"
#include <stdio.h>

// Calculate next command to send based on EMG and current limb position
void calculateCommands(hxRobotInfo *robotInfo, hxCommand *cmd, hxSensor *sensor, struct EMGData *emg, bool usingEMG, int counter);

#endif