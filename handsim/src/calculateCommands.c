// Calculate next controller command based on incoming EMG and current limb position
//
// Mikey Fernandez 04/11/2021

#include "../include/handsim/calculateCommands.h"

void calculateCommands(hxRobotInfo *robotInfo, hxCommand *cmd, hxSensor *sensor, struct EMGData *emg, bool usingEMG, int counter)
{
  int i;
  float a0, a1, a2;         // gains for EMG
  float al_ag, al_an;       // normalized EMG for a given joint
  float K0, K1, K2;         // stiffness gaings
  float K_net;              // net stiffness
  float pos;                // reference joint position
  int agonist, antagonist;  // indices for the two electrodes corresponding to a given joint
  float motorPos, motorRef; // current actual motor position, desired motor position based on muscle command **COMMANDS GIVE IN MOTOR SPACE
  float motorRange;
  float motorVel;

  int numMotors = robotInfo->motor_count;

  // arat - these are the default P gains for these joints
  // NOTE: With addition of controllable elbow joint, assumed default P gain of 100
  float K0_arr[23] = {100, 100, 100, 100, 30, 20, 8, 2, 20, 10, 5, 1, 10, 5, 1, 20, 10, 5, 1, 15, 10, 5, 1};
  // float K0_arr[numMotors] = {0}; K0_arr[2] = 100;
  float K1_arr[23] = {0};
  float K2_arr[23] = {0};

  float K = 10;
  float setpoint;

  float D_arr[23] = {10, 10, 10, 10, 10, 20, 8, 2, 20, 10, 5, 1, 10, 5, 1, 20, 10, 5, 1, 15, 10, 5, 10};

  if (usingEMG)
  {
    // ref_pos[i] = -(K0[i] + K1[i]*al_ag + K2[i]*al_an)*(a0[i] + (a1[i]*al_ag - a2[i]*al_an) - th[i]) + th[i]
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

    for (i = 0; i < numMotors; i++)
    {
      // find current motor position
      motorPos = sensor->motor_pos[i];
      motorVel = sensor->motor_vel[i];

      // find al_ag, al_an - TODO: make map between EMG sensors and their corresponding motors
      getElectrodes(i, &agonist, &antagonist);
      al_ag = getFilteredEMG(emg, agonist);    // flexion
      al_an = getFilteredEMG(emg, antagonist); // extension
      
      // find a0, a1, a2
      // a0 = 0;
      // a1 = robotInfo->motor_limit[i][1]; // this corresponds to the agonist muscle
      // a2 = robotInfo->motor_limit[i][0]; // this corresponds to the antagonist muscle
      motorRange = robotInfo->motor_limit[i][1] - robotInfo->motor_limit[i][0];
      setpoint = motorRange/2 + robotInfo->motor_limit[i][0];
      a0 = motorRange/2;
      a1 = motorRange;
      a2 = motorRange/2;

      // find K0, K1, and K2
      K0 = K0_arr[i];
      K1 = K1_arr[i];
      K2 = K2_arr[i];

      // calculate net stiffness
      K_net = K0 + K1*al_ag + K2*al_an;

      // calculate reference position of joint (motor)
      motorRef = a0 + (a1*al_ag - a2*al_an); // replaced subtraction in parenthesis with addition - a2 is negative

      // calculate reference position of joint[i]
      // K_net = 2;
      // pos = motorPos + K_net*(motorRef - motorPos) + D_arr[i]*motorVel;
      pos = motorPos + K*(motorRef - a0) + D_arr[i]*motorVel;

      // set in command struct
      cmd->ref_pos[i] = pos;
      // cmd->gain_pos[i] = K_net;
      cmd->gain_pos[i] = 1; // want the position gain just to be the default position gain of that motor

      if (i == 0)
      {
        printf("\t      a1: %06.2f\t      a2: %06.2f\n\t   al_ag: %06.4f\t   al_an: %06.4f\n\tmotorPos: %06.2f\tmotorRef: %06.2f\n\t  desPos: %06.2f\t   K_net: %06.2f\n\n", a1, a2, al_ag, al_an, motorPos, motorRef, pos, K_net);
      }
    }

    // Set what these commands are - we use a reference position and a position gain
    cmd->ref_pos_enabled = 1;
    cmd->gain_pos_enabled = 1;
    
  }
  else
  {
    int original = 1; // this is the original, built in example controller - sinusoidal wave

    if (original)
    {
      // Create a new command based on a sinusoidal wave.
      for (i = 0; i < numMotors; ++i)
      {
        // Set the desired position of this motor
        cmd->ref_pos[i] = (float)(350 * 0.5 *
          sin(0.05 * 2.0 * M_PI * counter * 0.01));
        // We could set a desired maximum velocity
        // cmd->ref_vel[i] = 1.0;
        // cmd->ref_vel_max[i] = 1.0;
        // We could set a desired controller position gain
        // cmd->gain_pos[i] = 1.0;
        // We could set a desired controller velocity gain
        // cmd->gain_vel[i] = 1.0;
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
    else // this is the 'experimental' region for handling hardcoded trajectories
    {
      float pos;
      float vel;
      // Create a new command based on a sinusoidal wave (hardcoded)
      for (i = 0; i < numMotors; ++i)
      {
        if (i == 2)
        {
          // Set the desired position of this motor
          pos = (float)(350 * 0.5 * sin(0.05 * 2.0 * M_PI * counter * 0.01));
          cmd->ref_pos[i] = pos;
          // vel = (float)(20 * 0.5 * sin(0.05 * 2.0 * M_PI * counter * 0.01));
          // cmd->ref_vel[i] = vel;
          // cmd->ref_vel_max[i] = 25;

          // printf("Position: %f\n", pos);

          // if (pos < 0)
          // {
          //   printf("\tNegative\n");
          // }
          // else
          // {
          //   printf("\tPositive\n");
          // }
          // We could set a desired maximum velocity
          // cmd->ref_vel[i] = 10.0;
          // cmd->ref_vel_max[i] = 200.0;
          // We could set a desired controller position gain
          // cmd->gain_pos[i] = 1.0;
          // We could set a desired controller velocity gain
          // cmd->gain_vel[i] = 1000.0;
        }
        else
        {
          cmd->ref_pos[i] = 0;
        }
      }
      // Indicate that the positions we set should be used.
      cmd->ref_pos_enabled = 1;
      // We're not setting it, so indicate that ref_vel should be ignored.
      cmd->ref_vel_enabled = 0;
      // We're not setting it, so indicate that ref_vel_max should be ignored.
      cmd->ref_vel_max_enabled = 1;
      // We're not setting it, so indicate that gain_pos should be ignored.
      cmd->gain_pos_enabled = 0;
      // We're not setting it, so indicate that gain_vel should be ignored.
      cmd->gain_vel_enabled = 1;
    }
  }
}