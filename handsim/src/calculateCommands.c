// Calculate next controller command based on incoming EMG and current limb position
//
// Mikey Fernandez 04/11/2021

#include "../include/handsim/calculateCommands.h"

float secondOrderDynamics(int motor, float T_des, struct EMGData *emg, struct secOrd *dynamics)
{
  // calculates command = wn^2/(s^2 + 2*zeta*wn*s + wn^2) [unit gain]

  float Wn = 2*M_PI*dynamics->freq_n; // desired natural frequency
  float b = 2*Wn;                     // 2*zeta*Wn, but critically damped so zeta = 1
  float k = Wn*Wn;                    // k = Wn^2

  float Ts = 1/emg->samplingFreq;
  float Ts2 = Ts*Ts;

  float k1 = (1/Ts2 + b/Ts + k);
  float k2 = -(2/Ts2 + b/Ts);
  float k3 = 1/Ts2;

  float prev = dynamics->prevT[motor];
  float prev2 = dynamics->prev2T[motor];

  float command = k/k1*T_des - k2/k1*prev - k3/k1*prev2;

  // if (motor == 0)
  // {
  //   printf("\nsecondOrderDynamics intermediates:\n\tWn: %f, b: %f, k: %f\n\tTs: %f, Ts2: %f\n\tk1: %f, k2: %f, k3: %f\n\tprev: %f, prev2: %f\n\tCommand: %f",
  //          Wn, b, k, Ts, Ts2, k1, k2, k3, prev, prev2, command);

  //   printf("T_des: %09.4f, prev: %09.4f, prev2: %09.4f, command: %09.4f\n", T_des, prev, prev2, command);
  // }
  
  // update previous commands in struct
  dynamics->prev2T[motor] = dynamics->prevT[motor];
  dynamics->prevT[motor] = command;

  return command;
}

float motorIntent(struct EMGData *emg, float *gains)
{  
  float sum = 0;
  int i;
  for (i = 0; i < emg->numElec; i++)
  {
    sum = sum + gains[i]*emg->muscleAct[i];
  }

  return sum;
}

float normalizeTorque(int motor, float weightedAct)
{
  float maxTorques[14] = {1, 1, 1, 0.5331, 1.1514, 1.1514, 1.1514, 1.1514, 1.1514, 1.1514, 1.1514, 1.1514, 1.1514, 1.1514};
  float minTorques[14] = {1, 1, 1, 0.7874, 1.3874, 1.3874, 1.3874, 1.3874, 1.3874, 1.3874, 1.3874, 1.3874, 1.3874, 1.3874};
  float normedTorque;

  if (weightedAct > 0)
  {
    normedTorque = weightedAct/maxTorques[motor];
  }
  else
  {
    normedTorque = weightedAct/minTorques[motor];
  }

  return normedTorque;
}

void calculateCommands(hxRobotInfo *robotInfo, hxCommand *cmd, hxSensor *sensor, struct EMGData *emg, struct secOrd *dynamics, bool usingEMG, int counter)
{
  int i;
  float a0, a1, a2;         // gains for EMG
  float al_ag, al_an;       // muscle activation (agonist, antagonist)
  float K0, K1, K2;         // stiffness gains
  float K_net;              // net stiffness
  int agonist, antagonist;  // indices for the two electrodes corresponding to a given joint
  float pos;                // reference motor position
  float motorPos, motorVT;  // current actual motor position, virtual trajectory motor position **COMMANDS GIVE IN MOTOR SPACE
  float motorRange;         // full range of motion of motor
  float motorVel;           // current actual motor angular velocity
  float setpoint;           // neutral setpoint for joint to return to
  float T_des;              // desired torque
  float K_active, K_pas;    // active torque gain, passive torque gain
  float th_0;               // spring resting position
  float gains[16] = {0};    // array of synergy gains
  float weightedAct;        // sum of muscle activations weighted by gains for that degree of freedom
  float T_active, T_pas;    // active torque contribution, passive torque contribution

  int numMotors = robotInfo->motor_count;

  float K_act_arr[14] = {0, 0, 0, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
  float K_pas_arr[14] = {0, 0, 0, 0, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5};

  // arat.world - these are the default P gains for these joints
  // NOTE: With addition of controllable elbow joint, assumed default P gain of 100
  //    float jointPgains[23] = {100, 100, 100, 100, 30, 20, 8, 2, 20, 10, 5, 1, 10, 5, 1, 20, 10, 5, 1, 15, 10, 5, 1};
  // float K0_arr[14] = {100, 100, 100, 100, 30, 20, 8, 2, 20, 10, 10, 10, 10, 5}; /* TUNE ME */
  // float K1_arr[14] = {0}; /* TUNE ME */
  // float K2_arr[14] = {0}; /* TUNE ME */
  // float D_arr[14] = {0};  /* TUNE ME */

  // float inc;
  // float delta;
  float th_u, th_l;

  if (usingEMG)
  {
    float numElec = emg->numElec;

    /* CONTROL LAW */
    // T_des[i] = K_active[i]*dot(gains[i]*muscleAct)/TorqueNorm[i] - K_passive[i]*(motorPos[i] - th_0[i])
    //
    //   where:
    //     T_des[i] is the desired motor torque (position) input to motor i
    //     K_active[i] is the active torque gain for motor i
    //     gains[i] are the 16 synergy gains for the DoF controlled by motor i
    //     muscleAct are the current muscle activation values
    //     TorqueNorm[i] is the normalization factor from the synergy motor intent for motor i
    //     K_passive[i] is the pasive spring stiffness for motor i (for opening the hand)
    //     motorPos[i] is the position of motor i
    //     th_0[i] is the reference position of motor i for the passive spring

    for (i = 0; i < numMotors; i++)
    {
      // find current motor position and velocity
      motorPos = sensor->motor_pos[i];
      motorVel = sensor->motor_vel[i];

      // get the reference position for the passive spring
      // th_0 = robotInfo->motor_limit[i][0];
      th_0 = 0;

      // get active and passive torque gains
      K_active = K_act_arr[i];
      K_pas = K_pas_arr[i];

      // get synergy gains for the DoF this motor controls
      getGains(i, gains, numElec);
      
      // get motor intent
      weightedAct = motorIntent(emg, gains);
      T_active = K_active*normalizeTorque(i, weightedAct);
      T_pas = -K_pas*(motorPos - th_0);

      T_des = T_active + T_pas;

      // calculate position by using second order dynamics
      pos = secondOrderDynamics(i, T_des, emg, dynamics); 

      // set in command struct
      cmd->ref_pos[i] = pos;
      cmd->gain_pos[i] = 1; // position error gain needs to be 1 for force to be equal to desired torque

      if (i == 5)
      {
        // printMuscleActivation(emg->muscleAct);
        printf("Motor: %d\nActive Torque: %f\nPassive Torque: %f\nDesired Position: %f\nCurrent Position: %f\n", i, T_active, T_pas, pos, motorPos);
        // printf("Gains: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
        // gains[0], gains[1], gains[2], gains[3], gains[4], gains[5], gains[6], gains[7], gains[8], gains[9], gains[10], gains[11], gains[12], gains[13], gains[14], gains[15]);
        // printf("Motor Intent: %f\n", weightedAct);
      }

      /* OLD IMPEDANCE CONTROLLER */
      // T_des[i] = (K0[i] + K1[i]*al_ag + K2[i]*al_an)*(a0[i] + (a1[i]*al_ag - a2[i]*al_an) - th[i]) - D[i]*om[i]
      //     
      //     Starting point:
      //         a0 = 0
      //         a1 = full flex angle      (robotInfo->joint_limit[i][0])
      //         a2 = full extension angle (robotInfo->joint_limit[i][1])
      //
      //         0 <= al_ag, al_an <= 1 (normalized EMG)
      //
      //         th[i] = sensor->motor_pos[i]
      //         om[i] = sensor->motor_vel[i]
      //
      //         K0[i] is the constant stiffness of the joint
      //         K1[i] must be tuned (contribution of flexor)
      //         K2[i] must be tuned (contribution of extensor)

      // find al_ag, al_an - map between EMG electrodes and their corresponding motors in 'EMGStruct.c'
      // getElectrodes(i, &agonist, &antagonist);
      // al_ag = getFilteredEMG(emg, agonist);    // flexion
      // al_an = getFilteredEMG(emg, antagonist); // extension

      // // find a0, a1, a2
      // a0 = 0;
      // a1 = robotInfo->motor_limit[i][1]; // this corresponds to the agonist muscle
      // a2 = robotInfo->motor_limit[i][0]; // this corresponds to the antagonist muscle
      // motorRange = robotInfo->motor_limit[i][1] - robotInfo->motor_limit[i][0];
      // setpoint = motorRange/2 + robotInfo->motor_limit[i][0]; // halfway through full RoM
      // // a0 = setpoint;
      // // a1 = motorRange/2;
      // // a2 = motorRange/2;
      // // these settings for a_i would have limb at neutral when no muscle activity and
      // // fully flexed/extended for max agonist/antagonist activation

      // // find K0, K1, and K2
      // K0 = 1;
      // K1 = 0;
      // K2 = 0;
      // // K0 = K0_arr[i];
      // // K1 = K1_arr[i];
      // // K2 = K2_arr[i];

      // // calculate net stiffness
      // K_net = K0 + K1*al_ag + K2*al_an;

      // // calculate virtual tracjetory position of motor
      // motorVT = a0 + (a1*al_ag - a2*al_an);

      // calculate reference position of motor[i]
      // pos = motorPos + K*(motorVT - a0) - D_arr[i]*motorVel;

      // calculate torque based on impedance controller + passive spring
      // T_des = K_net*(motorVT - motorPos) - D_arr[i]*motorVel;
      // T_des = K_net*(motorVT - motorPos) - D_arr[i]*motorVel + K_pas*(motorPos - robotInfo->motor_limit[i][0]);

      // if (i == 0)
      // {
      //   // printf("\t   al_ag: %06.4f\t   al_an: %06.4f\n\t   T_des: %06.2f\n\tmotorPos: %06.2f\tmotorVT:  %06.2f\n\t  desPos: %06.2f\t   K_net: %06.2f\n\n",
      //   //        al_ag, al_an, T_des, motorPos, motorVT, pos, K_net);
      // }
    }

    // for (i = 0; i < numMotors; i++)
    // {
    //   // find current motor position and velocity
    //   motorPos = sensor->motor_pos[i];
    //   motorVel = sensor->motor_vel[i];

    //   // find al_ag, al_an - map between EMG electrodes and their corresponding motors in 'EMGStruct.c'
    //   getElectrodes(i, &agonist, &antagonist);
    //   al_ag = getFilteredEMG(emg, agonist);    // flexion
    //   al_an = getFilteredEMG(emg, antagonist); // extension

    //   delta = al_ag - al_an; // difference in activation

    //   th_l = robotInfo->motor_limit[i][0]; // this corresponds to the agonist muscle
    //   th_u = robotInfo->motor_limit[i][1]; // this corresponds to the antagonist muscle
    //   motorRange = th_u - th_l;

    //   // int pair = agonist/2; // TODO: make me a nice function instead
    //   // float maxD = emg->deltas[pair];
    //   // float minD = emg->deltas[8 + pair];

    //   // float K1 = (th_u - th_l)/(maxD - minD);
    //   // float K2 = (-minD*th_u + maxD*th_l)/(maxD - minD);
      
    //   if (i == 0)
    //   {
    //     delta = delta + 0.075;
    //     if (delta > 0)
    //     {
    //       inc = 500;
    //     }
    //     else
    //     {
    //       inc = 500;
    //     }
    //   }
    //   else
    //   {
    //     delta = delta + 0.02;
    //     if (delta > 0)
    //     {
    //       inc = 200;
    //     }
    //     else
    //     {
    //       inc = 400;
    //     }
    //   }

    //   pos = motorPos + inc*delta*motorRange/100;

    //   if (pos < th_l)
    //   {
    //     pos = th_l;
    //   }
    //   else if (pos > th_u)
    //   {
    //     pos = th_u;
    //   }

    //   if (i == 4)
    //   {
    //     // printf("Position commanded: %f\n", pos);
    //     printf("Delta: %f\n", delta);
    //     printf("\n");
    //   }

    //   // set in command struct
    //   cmd->ref_pos[i] = pos;
    //   cmd->gain_pos[i] = 1; // position error gain needs to be 1 for force to be equal to desired torque

    //   // force wristx, wristy, wristz to stay at 0
    //   if (i == 3 || i == 2 || i == 1) 
    //   {
    //     cmd->ref_pos[i] = 0;
    //   }
    // }

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