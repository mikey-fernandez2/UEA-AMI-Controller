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
  // float K0_arr[22] = {0}; K0_arr[2] = 100;
  float K1_arr[22] = {0};
  float K2_arr[22] = {0};

  float motorPos, motorRef;

  // motorMap[0][i] is the electrode corresponding to the agonist muscle for motor i
  // motorMap[1][i] is the electrode corresponding to the antagonist muscle for motor i
  // int motorMap[2][22] = {{4, 2, 0, 6, 6, 6, 6, 8, 8, 8, 8, 10, 10, 10, 12, 12, 12, 12, 14, 14, 14, 14},
  //                        {5, 3, 1, 7, 7, 7, 7, 9, 9, 9, 9, 11, 11, 11, 13, 13, 13, 13, 15, 15, 15, 15}};
  int motorMap[22][2] = {{4, 5}, {2, 3}, {0, 1}, {6, 7}, {6, 7}, {6, 7}, {8, 9}, {8, 9}, {8, 9}, {8, 9},
                         {10, 11}, {10, 11}, {10, 11}, {12, 13}, {12, 13}, {12, 13}, {12, 13}, {14, 15},
                         {14, 15}, {14, 15}, {14, 15}};

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
      if (i == 2)
      {
        // find current joint position
        // jointPos = sensor->joint_pos[i];

        // find current motor position
        motorPos = sensor->motor_pos[i];

        /* FOR SOME REASON, NEGATIVE POSITION FOR JOINT 2 SEEMS TO BE EXTENSION */
        /* COMMANDS ARE GIVEN FOR DESIRED MOTOR POSITION - MAGNITUDES WILL BE FAR LARGER THAN EXPECTED */

        // find al_ag, al_an - TODO: make map between EMG sensors and their corresponding motors
        agonist = motorMap[i][0];    al_ag = getNormedEMG(emg, agonist); // flexion
        antagonist = motorMap[i][1]; al_an = getNormedEMG(emg, antagonist); // extension
        
        // find a0, a1, a2
        a0 = 0;
        // a1 = robotInfo->joint_limit[i][0]; // this corresponds to the agonist muscle
        // a2 = robotInfo->joint_limit[i][1]; // this corresponds to the antagonist muscle
        a1 = robotInfo->motor_limit[i][1]; // this corresponds to the agonist muscle
        a2 = robotInfo->motor_limit[i][0]; // this corresponds to the antagonist muscle

        // find K0, K1, and K2
        K0 = K0_arr[i];
        K1 = K1_arr[i];
        K2 = K2_arr[i];

        // calculate net stiffness
        K_net = K0 + K1*al_ag + K2*al_an;

        // calculate reference position of joint
        // jointRefPos = a0 + (a1*al_ag + a2*al_an); // replaced subtraction in parenthesis with addition - a2 is negative
        motorRef = a0 + (a1*al_ag + a2*al_an); // replaced subtraction in parenthesis with addition - a2 is negative

        // calculate reference position of joint[i]
        // pos = K_net*(jointRefPos - jointPos) + jointPos;
        pos = 2*(motorRef - motorPos) + motorPos;
        // pos = 1*(jointRefPos - jointPos) + jointPos;

        // set in command struct
        cmd->ref_pos[i] = pos;
        cmd->gain_pos[i] = K_net;

        // printf("\t      a1: %06.2f\t\t   a2: %06.2f\n\t   al_ag: %06.2f\t\tal_an: %06.2f\n\tjointPos: %06.2f\t  jointRefPos: %06.2f\n\t  desPos: %06.2f\t\tK_net: %06.2f\n\n", a1, a2, al_ag, al_an, jointPos, jointRefPos, pos, K_net);
        printf("\t      a1: %06.2f\t      a2: %06.2f\n\t   al_ag: %06.4f\t   al_an: %06.4f\n\tmotorPos: %06.2f\tmotorRef: %06.2f\n\t  desPos: %06.2f\t   K_net: %06.2f\n\n", a1, a2, al_ag, al_an, motorPos, motorRef, pos, K_net);
      }
      else
      {
        cmd->ref_pos[i] = 0;
        cmd->gain_pos[i] = K0_arr[i];
      }
    }

    // Set what these commands are - we use a reference position and a position gain
    cmd->ref_pos_enabled = 1;
    cmd->gain_pos_enabled = 1;
    
  }
  else
  {
    float pos;
    // Create a new command based on a sinusoidal wave (hardcoded)
    for (i = 0; i < robotInfo->motor_count; ++i)
    {
      if (i == 2)
      {
        // Set the desired position of this motor
        pos = (float)(350 * 0.5 * sin(0.05 * 2.0 * M_PI * counter * 0.01));
        cmd->ref_pos[i] = pos;

        printf("Position: %f\n", pos);

        if (pos < 0)
        {
          printf("\tNegative\n");
        }
        else
        {
          printf("\tPositive\n");
        }
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
    cmd->ref_vel_max_enabled = 0;
    // We're not setting it, so indicate that gain_pos should be ignored.
    cmd->gain_pos_enabled = 0;
    // We're not setting it, so indicate that gain_vel should be ignored.
    cmd->gain_vel_enabled = 0;
  }
}