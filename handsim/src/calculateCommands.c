// C file to read in commands

#ifdef _WIN32
#include <windows.h>
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>
#include <haptix/comm/haptix.h>
#ifdef _WIN32
#include <windows.h>
#endif
#include "../include/handsim/calculateCommands.h"

void calculateCommands(hxRobotInfo *robotInfo, hxCommand *cmd, hxSensor *sensor)
{
    // int i = 0;

    // // Create a new command based on a sinusoidal wave.
    // for (i = 0; i < robotInfo->motor_count; ++i)
    // {
    //   // Set the desired position of this motor
    //   cmd->ref_pos[i] = (float)(350 * 0.5 *
    //     sin(0.05 * 2.0 * M_PI * 0.01));
    //   // We could set a desired maximum velocity
    //   // cmd.ref_vel[i] = 1.0;
    //   // cmd.ref_vel_max[i] = 1.0;
    //   // We could set a desired controller position gain
    //   // cmd.gain_pos[i] = 1.0;
    //   // We could set a desired controller velocity gain
    //   // cmd.gain_vel[i] = 1.0;
    // }
    // // Indicate that the positions we set should be used.
    // cmd->ref_pos_enabled = 1;
    // // We're not setting it, so indicate that ref_vel should be ignored.
    // cmd->ref_vel_enabled = 0;
    // // We're not setting it, so indicate that ref_vel_max should be ignored.
    // cmd->ref_vel_max_enabled = 0;
    // // We're not setting it, so indicate that gain_pos should be ignored.
    // cmd->gain_pos_enabled = 0;
    // // We're not setting it, so indicate that gain_vel should be ignored.
    // cmd->gain_vel_enabled = 0;

    return;
}