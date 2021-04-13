// C file to read in commands

// #ifdef _WIN32
// #include <windows.h>
// #define _USE_MATH_DEFINES
// #endif
// #include <math.h>
// #include <signal.h>
// #include <stdio.h>
// #include <time.h>
// #include <haptix/comm/haptix.h>
// #ifdef _WIN32
// #include <windows.h>
// #endif
#include "../include/handsim/calculateCommands.h"

void calculateCommands(hxRobotInfo *robotInfo, hxCommand *cmd, hxSensor *sensor, struct EMGData *emg, bool usingEMG, int counter)
{
  int i = 0;

  if (usingEMG)
  {
    // ref_pos[i] = (K0[i] + K1[i]*al_ag + K2[i]*al_an)*(a0[i] + (a1[i]*al_ag - a2[i]*al_an) - th[i]) + th[i]
    //     Starting point:
    //         a0 = 0
    //         a1 = full flex angle      (robotInfo.joint_limit[i][0])
    //         a2 = full extension angle (robotInfo.joint_limit[i][1])
    //
    //     0 <= al_ag, al_an <= 1 (normalized EMG)
    //
    //     th = sensor->joint_pos[i]
    //
    //     TODO: figure out how to calculate K[i]

    // calculateCommands(&robotInfo, &cmd, &sensor);

    for (i = 0; i < robotInfo->motor_count; ++i)
    {
      // Set the desired position of this motor
      if (i == 2)
        cmd->ref_pos[i] = 0;
      else
        cmd->ref_pos[i] = 0.0;
      // cmd->ref_pos[i] = 0.0;
      // cmd->ref_vel_max[i] = 5.0;
      // We could set a desired controller position gain
      // cmd->gain_pos[i] = 1.0;
      // We could set a desired controller velocity gain
      // cmd->gain_vel[i] = 1.0;
    }
    cmd->ref_pos_enabled = 1;
    cmd->ref_vel_enabled = 0;
    cmd->ref_vel_max_enabled = 0;
    cmd->gain_pos_enabled = 0;
    cmd->gain_vel_enabled = 0;
  }
  else
  {
    // Create a new command based on a sinusoidal wave (hardcoded)
    for (i = 0; i < robotInfo->motor_count; ++i)
    {
      // Set the desired position of this motor
      cmd->ref_pos[i] = (float)(350 * 0.5 *
        sin(0.05 * 2.0 * M_PI * counter * 0.01));
      // We could set a desired maximum velocity
      // cmd->ref_vel[i] = 10.0;
      cmd->ref_vel_max[i] = 200.0;
      // We could set a desired controller position gain
      // cmd->gain_pos[i] = 1.0;
      // We could set a desired controller velocity gain
      cmd->gain_vel[i] = 1000.0;
    }
    // Indicate that the positions we set should be used.
    cmd->ref_pos_enabled = 1;
    // We're not setting it, so indicate that ref_vel should be ignored.
    cmd->ref_vel_enabled = 0;
    // We're not setting it, so indicate that ref_vel_max should be ignored.
    cmd->ref_vel_max_enabled = 0;
    // We're not setting it, so indicate that gain_pos should be ignored.
    cmd->gain_pos_enabled = 0;
    // We're not setting it, so indicate that gain_vel should be ignored.
    cmd->gain_vel_enabled = 0;
  }
}