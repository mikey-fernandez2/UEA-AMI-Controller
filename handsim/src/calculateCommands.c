// Calculate next controller command based on incoming EMG and current limb position
//
// Mikey Fernandez 04/11/2021

#include "../include/handsim/calculateCommands.h"

void calculateCommands(hxRobotInfo *robotInfo, hxCommand *cmd, hxSensor *sensor, struct EMGData *emg, bool usingEMG, int counter)
{
  int i = 0;
  float jointPos;           // current actual joint position
  float a0, a1, a2;         // gains for EMG
  float al_ag, al_an;       // normalized EMG for a given joint
  float K0, K1, K2;         // stiffness gaings
  float K_net;              // net stiffness
  float jointRefPos;        // desired joint position from EMG
  float pos;                // reference joint position
  int agonist, antagonist;  // indices for the two electrodes corresponding to a given joint

  // arat - these are the default P gains for these joints
  float K0_arr[22] = {100, 100, 100, 30, 20, 8, 2, 20, 10, 5, 1, 10, 5, 1, 20, 10, 5, 1, 15, 10, 5, 1};
  float K1_arr[22] = {0};
  float K2_arr[22] = {0};

  if (usingEMG)
  {
    // ref_pos[i] = (K0[i] + K1[i]*al_ag + K2[i]*al_an)*(a0[i] + (a1[i]*al_ag - a2[i]*al_an) - th[i]) + th[i]
    //     
    //     Starting point:
    //         a0 = 0
    //         a1 = full flex angle      (robotInfo->joint_limit[i][0])
    //         a2 = full extension angle (robotInfo->joint_limit[i][1])
    //
    //         0 <= al_ag, al_an <= 1 (normalized EMG)
    //
    //         th = sensor->joint_pos[i]
    //
    //         K0[i] is the original kp this arm uses
    //         K1[i] must be tuned
    //         K2[i] must be tuned

    for (i = 0; i < robotInfo->motor_count; i++)
    {
      // find current joint position
      jointPos = sensor->motor_pos[i];

      // find a0, a1, a2
      a0 = 0;
      a1 = robotInfo->joint_limit[i][0];
      a2 = robotInfo->joint_limit[i][1];

      // find al_ag, al_an - TODO: make map between EMG sensors and their corresponding motors
      agonist = 0; antagonist = 0;
      al_ag = getNormedEMG(emg, agonist); //->normedEMG[agonist];
      al_an = getNormedEMG(emg, antagonist); //emg->normedEMG[antagonist];

      // find K0, K1, and K2
      K0 = K0_arr[i];
      K1 = K1_arr[i];
      K2 = K2_arr[i];

      // calculate net stiffness
      K_net = K0 + K1*al_ag + K2*al_an;

      // calculate reference position of joint
      jointRefPos = a0 + (a1*al_ag - a2*al_an);

      // calculate reference position of joint[i]
      pos = K_net*(jointRefPos - jointPos) + jointPos;

      // set in command struct
      cmd->ref_pos[i] = pos;
      cmd->gain_pos[i] = K_net;
    }

    // Set what these commands are - we use a reference position and a position gain
    cmd->ref_pos_enabled = 1;
    cmd->gain_pos_enabled = 1;

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