// Calculate next controller command based on incoming EMG and current limb position
//
// Mikey Fernandez 04/11/2021

#include "../include/handsim/calculateCommands.h"

float secondOrderDynamics(int motor, float T_des, struct EMGData *emg, struct prevCom *prior)
{
  float Wn = 2*M_PI*emg->freq_n;
  float b = 2*Wn;
  float k = Wn*Wn;

  float Ts = 1/emg->samplingFreq;
  float Ts2 = Ts*Ts;

  float k1 = (1/Ts2 + b/Ts + k);
  float k2 = -(2/Ts2 + b/Ts);
  float k3 = 1/Ts2;

  float prev = prior->prevT[motor];
  float prev2 = prior->prev2T[motor];

  return k*Ts2*T_des - k2/k1*prev - k3/k1*prev2;
}

void calculateCommands(hxRobotInfo *robotInfo, hxCommand *cmd, hxSensor *sensor, struct EMGData *emg, struct prevCom *prior, bool usingEMG, int counter)
{
  int i;
  float a0, a1, a2;         // gains for EMG
  float al_ag, al_an;       // muscle activation (agonist, antagonist)
  float K0, K1, K2;         // stiffness gains
  float K_net;              // net stiffness
  int agonist, antagonist;  // indices for the two electrodes corresponding to a given joint
  float pos;                // reference motor position
  float motorPos, motorRef; // current actual motor position, virtual trajectory motor position **COMMANDS GIVE IN MOTOR SPACE
  float motorRange;         // full range of motion of motor
  float motorVel;           // current actual motor angular velocity
  float K = 10;             // 
  float setpoint;           // neutral setpoint for joint to return to

  int numMotors = robotInfo->motor_count;

  // arat.world - these are the default P gains for these joints
  // NOTE: With addition of controllable elbow joint, assumed default P gain of 100
  //    float jointPgains[23] = {100, 100, 100, 100, 30, 20, 8, 2, 20, 10, 5, 1, 10, 5, 1, 20, 10, 5, 1, 15, 10, 5, 1};
  float K0_arr[14] = {0}; /* TUNE ME */
  float K1_arr[14] = {0}; /* TUNE ME */
  float K2_arr[14] = {0}; /* TUNE ME */
  float D_arr[14] = {0};  /* TUNE ME */

  float inc;
  float delta;
  float th_u, th_l;

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

    // for (i = 0; i < numMotors; i++)
    // {
    //   // find current motor position and velocity
    //   motorPos = sensor->motor_pos[i];
    //   motorVel = sensor->motor_vel[i];

    //   // find al_ag, al_an - map between EMG electrodes and their corresponding motors in 'EMGStruct.c'
    //   getElectrodes(i, &agonist, &antagonist);
    //   al_ag = getFilteredEMG(emg, agonist);    // flexion
    //   al_an = getFilteredEMG(emg, antagonist); // extension
      
    //   // find a0, a1, a2
    //   // a0 = 0;
    //   // a1 = robotInfo->motor_limit[i][1]; // this corresponds to the agonist muscle
    //   // a2 = robotInfo->motor_limit[i][0]; // this corresponds to the antagonist muscle
    //   motorRange = robotInfo->motor_limit[i][1] - robotInfo->motor_limit[i][0];
    //   setpoint = motorRange/2 + robotInfo->motor_limit[i][0]; // halfway through full RoM
    //   a0 = motorRange/2; /* TUNE ME */
    //   a1 = motorRange;   /* TUNE ME */
    //   a2 = motorRange/2; /* TUNE ME */

    //   // find K0, K1, and K2
    //   K0 = K0_arr[i];
    //   K1 = K1_arr[i];
    //   K2 = K2_arr[i];

    //   // calculate net stiffness
    //   K_net = K0 + K1*al_ag + K2*al_an;

    //   // calculate reference position of motor
    //   motorRef = a0 + (a1*al_ag - a2*al_an);

    //   // calculate reference position of motor[i]
    //   // pos = motorPos + K_net*(motorRef - motorPos) - D_arr[i]*motorVel;
    //   pos = motorPos + K*(motorRef - a0) - D_arr[i]*motorVel;

    //   // set in command struct
    //   cmd->ref_pos[i] = pos;
    //   cmd->gain_pos[i] = 1; // position error gain needs to be 1 for force to be equal to desired torque

    //   if (i == 0)
    //   {
    //     printf("\t      a1: %06.2f\t      a2: %06.2f\n\t   al_ag: %06.4f\t   al_an: %06.4f\n\tmotorPos: %06.2f\tmotorRef: %06.2f\n\t  desPos: %06.2f\t   K_net: %06.2f\n\n",
    //            a1, a2, al_ag, al_an, motorPos, motorRef, pos, K_net);
    //   }
    // }

    for (i = 0; i < numMotors; i++)
    {
      // find current motor position and velocity
      motorPos = sensor->motor_pos[i];
      motorVel = sensor->motor_vel[i];

      // find al_ag, al_an - map between EMG electrodes and their corresponding motors in 'EMGStruct.c'
      getElectrodes(i, &agonist, &antagonist);
      al_ag = getFilteredEMG(emg, agonist);    // flexion
      al_an = getFilteredEMG(emg, antagonist); // extension

      delta = al_ag - al_an; // difference in activation

      th_l = robotInfo->motor_limit[i][0]; // this corresponds to the agonist muscle
      th_u = robotInfo->motor_limit[i][1]; // this corresponds to the antagonist muscle
      motorRange = th_u - th_l;

      // int pair = agonist/2; // TODO: make me a nice function instead
      // float maxD = emg->deltas[pair];
      // float minD = emg->deltas[8 + pair];

      // float K1 = (th_u - th_l)/(maxD - minD);
      // float K2 = (-minD*th_u + maxD*th_l)/(maxD - minD);
      
      if (i == 0)
      {
        delta = delta + 0.075;
        if (delta > 0)
        {
          inc = 500;
        }
        else
        {
          inc = 500;
        }
      }
      else
      {
        delta = delta + 0.02;
        if (delta > 0)
        {
          inc = 200;
        }
        else
        {
          inc = 400;
        }
      }

      pos = motorPos + inc*delta*motorRange/100;

      if (pos < th_l)
      {
        pos = th_l;
      }
      else if (pos > th_u)
      {
        pos = th_u;
      }

      if (i == 4)
      {
        // printf("Position commanded: %f\n", pos);
        printf("Delta: %f\n", delta);
        printf("\n");
      }

      // set in command struct
      cmd->ref_pos[i] = pos;
      cmd->gain_pos[i] = 1; // position error gain needs to be 1 for force to be equal to desired torque

      // force wristx, wristy, wristz to stay at 0
      if (i == 3 || i == 2 || i == 1) 
      {
        cmd->ref_pos[i] = 0;
      }
    }

    // update previous commands stored
    prior->prev2T[i] = prior->prevT[i];
    prior->prevT[i] = cmd->ref_pos[i];

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
        th_l = robotInfo->motor_limit[i][0]; // this corresponds to the agonist muscle
        th_u = robotInfo->motor_limit[i][1]; // this corresponds to the antagonist muscle
        motorRange = th_u - th_l;

        cmd->ref_pos[i] = (float)(motorRange/2 * 
          sin(0.05 * 2.0 * M_PI * counter * 0.01)) + motorRange/2;
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