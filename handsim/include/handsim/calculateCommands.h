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

struct secOrd {
    float prevT[14];  // previous torque commands
    float prev2T[14]; // 2 time steps previous command
    // there are 14 motors to control

    float freq_n; // natural frequency for limb movement, Hz
};

// calculate critically damped second order dynamics output for a given motor and input torque
float secondOrderDynamics(int motor, float T_des, struct EMGData *emg, struct secOrd *dynamics);

// calculates the weighted sum of all muscle activations to get the motor intent
float motorIntent(struct EMGData *emg, float *gains);

// normalize the weighted sum of muscle activations
float normalizeTorque(int motor, float weightedAct);

// Calculate next command to send based on EMG and current limb position
void calculateCommands(hxRobotInfo *robotInfo, hxCommand *cmd, hxSensor *sensor, struct EMGData *emg, struct secOrd *dynamics, bool usingEMG, int counter);

#endif