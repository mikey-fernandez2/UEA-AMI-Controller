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

struct prevCom {
    float prevT[14];  // previous torque commands
    float prev2T[14]; // 2 time steps previous command
    // there are 14 motors to control
};

// calculate critically damped second order dynamics output
float secondOrderDynamics(int motor, float T_des, struct EMGData *emg, struct prevCom *prior);

// Calculate next command to send based on EMG and current limb position
void calculateCommands(hxRobotInfo *robotInfo, hxCommand *cmd, hxSensor *sensor, struct EMGData *emg, struct prevCom *prior, bool usingEMG, int counter);

#endif