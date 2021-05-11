/* Copyright (C) 2014-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "../include/handsim/printFunctions.h"

void printState(const hxRobotInfo *_robotInfo, const hxSensor *_sensor)
{
  // int i;

  // printf("Time: %d.%09d\n",
  //   _sensor->time_stamp.sec, _sensor->time_stamp.nsec);

  // printf("Motors:\n");
  // for (i = 0; i < _robotInfo->motor_count; ++i)
  // {
  //   printf("\tMotor %d\n", i);
  //   printf("\t\tPosition: %f rads\n", _sensor->motor_pos[i]);
  //   printf("\t\tVelocity: %f rads/sec\n", _sensor->motor_vel[i]);
  //   printf("\t\tTorque: %f N*m\n" , _sensor->motor_torque[i]);
  // }

  // printf("Joints:\n");
  // for (i = 0; i < _robotInfo->joint_count; ++i)
  // {
  //   printf("\tJoint %d\n", i);
  //   printf("\t\tPosition: %f rads\n", _sensor->joint_pos[i]);
  //   printf("\t\tVelocity: %f rads/sec\n", _sensor->joint_vel[i]);
  // }

  // printf("Contact sensors:\n");
  // for (i = 0; i < _robotInfo->contact_sensor_count; ++i)
  // {
  //   printf("\t# %d\n", i);
  //   printf("\t\tvalue: %f N.\n", _sensor->contact[i]);
  // }

  // printf("IMUs:\n");
  // for (i = 0; i < _robotInfo->imu_count; ++i)
  // {
  //   printf("\t# %d\n", i);
  //   printf("\t\tLinear acceleration: (%f, %f, %f) m/sec2\n",
  //     _sensor->imu_linear_acc[i][0], _sensor->imu_linear_acc[i][1],
  //     _sensor->imu_linear_acc[i][2]);
  //   printf("\t\tAngular velocity: (%f, %f, %f) rads/sec\n",
  //     _sensor->imu_angular_vel[i][0], _sensor->imu_angular_vel[i][1],
  //     _sensor->imu_angular_vel[i][2]);
  // }

  // printf("\n");

  return;
}

//////////////////////////////////////////////////
void printRobotInfo(const hxRobotInfo *_robotInfo)
{
  printf("Robot information received:\n");
  printf("Num motors: %d\n", _robotInfo->motor_count);
  printf("Num joints: %d\n", _robotInfo->joint_count);
  printf("Num contact sensors: %d\n", _robotInfo->contact_sensor_count);
  printf("Num IMUs: %d\n", _robotInfo->imu_count);
  printf("Update rate: %f\n", _robotInfo->update_rate);
  printf("Actuated joint limits: \n");

  // Print joint limits.
  int i;
  for (i = 0; i < _robotInfo->motor_count; ++i)
  {
    printf("\tJoint associated to motor %d:\n", i);
    printf("\t\t Min: %f rads.\n", _robotInfo->joint_limit[i][0]); // lower limit
    printf("\t\t Max: %f rads.\n", _robotInfo->joint_limit[i][1]); // upper limit
  }

  printf("\n");
}

//////////////////////////////////////////////////
// Print next command to be sent
void printCommand(const hxRobotInfo *_robotInfo, const hxCommand *_cmd)
{
  printf("Command received:\n");

  // Print command settings.
  printf("\tref_pos_enabled: %d\n",     _cmd->ref_pos_enabled);
  printf("\tref_vel_enabled: %d\n",     _cmd->ref_vel_enabled);
  printf("\tref_vel_max_enabled: %d\n", _cmd->ref_vel_max_enabled);
  printf("\tgain_pos_enabled: %d\n",    _cmd->gain_pos_enabled);
  printf("\tgain_vel_enabled: %d\n",    _cmd->gain_vel_enabled);

  // Print individual motor commands.
  int i;
  printf("\n\tMotors:\n");
  for (i = 0; i < _robotInfo->motor_count; ++i)
  {
    printf("\t\tMotor %d\n", i);
    printf("\t\t\tref_pos: %f rads\n",         _cmd->ref_pos[i]);
    printf("\t\t\tref_vel: %f rads/sec\n",     _cmd->ref_vel[i]);
    printf("\t\t\tref_vel_max: %f rads/sec\n", _cmd->ref_vel_max[i]);
    printf("\t\t\tgain_pos: %f Nm/rad\n",      _cmd->gain_pos[i]);
    printf("\t\t\tgain_vel: %f Nm*sec/rad\n",  _cmd->gain_vel[i]);
  }

  printf("\n");
}

// Print next command to be sent
void printCommandMotor(const hxRobotInfo *_robotInfo, const hxCommand *_cmd, int i)
{
  printf("Command for motor %d:\n", i);

  printf("\tref_pos:     %07.2f rads\n",         _cmd->ref_pos[i]);
  printf("\tref_vel:     %07.2f rads/sec\n",     _cmd->ref_vel[i]);
  printf("\tref_vel_max: %07.2f rads/sec\n", _cmd->ref_vel_max[i]);
  printf("\tgain_pos:    %07.2f Nm/rad\n",      _cmd->gain_pos[i]);
  printf("\tgain_vel:    %07.2f Nm*sec/rad\n",  _cmd->gain_vel[i]);

  printf("\n");
}

//////////////////////////////////////////////////
// Print EMG struct
void printEMGData(const struct EMGData *emg)
{
  printf("EMG data received:\n");

  printf("\tRunning time: %08.2f ms\n", emg->OS_time);
  printf("\tTrigger: %d\n", emg->trigger);
  printf("\tSwitch1: %d\n", emg->switch1);
  printf("\tSwitch2: %d\n", emg->switch2);

  printf("\tReceiving Frequency: %f\n", emg->samplingFreq);

  printf("\tRaw EMG:\n");
  printf("\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n",
    emg->rawEMG[0], emg->rawEMG[4], emg->rawEMG[8],  emg->rawEMG[12],
    emg->rawEMG[1], emg->rawEMG[5], emg->rawEMG[9],  emg->rawEMG[13],
    emg->rawEMG[2], emg->rawEMG[6], emg->rawEMG[10], emg->rawEMG[14],
    emg->rawEMG[3], emg->rawEMG[7], emg->rawEMG[11], emg->rawEMG[15]);

  printf("\tNormed EMG:\n");
  printf("\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n",
    emg->normedEMG[0], emg->normedEMG[4], emg->normedEMG[8],  emg->normedEMG[12],
    emg->normedEMG[1], emg->normedEMG[5], emg->normedEMG[9],  emg->normedEMG[13],
    emg->normedEMG[2], emg->normedEMG[6], emg->normedEMG[10], emg->normedEMG[14],
    emg->normedEMG[3], emg->normedEMG[7], emg->normedEMG[11], emg->normedEMG[15]);

  printf("\tMuscle Activation:\n");
  printf("\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n",
    emg->muscleAct[0], emg->muscleAct[4], emg->muscleAct[8],  emg->muscleAct[12],
    emg->muscleAct[1], emg->muscleAct[5], emg->muscleAct[9],  emg->muscleAct[13],
    emg->muscleAct[2], emg->muscleAct[6], emg->muscleAct[10], emg->muscleAct[14],
    emg->muscleAct[3], emg->muscleAct[7], emg->muscleAct[11], emg->muscleAct[15]);

  printf("\n");
}

// Print EMG normalization factors
void printEMGNorms(float *norms)
{
  printf("EMG normalization factors:\n");
  printf("\tMax: \n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n",
    norms[0], norms[4], norms[8],  norms[12],
    norms[1], norms[5], norms[9],  norms[13],
    norms[2], norms[6], norms[10], norms[14],
    norms[3], norms[7], norms[11], norms[15]);

  printf("\tMin: \n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n",
    norms[16], norms[17], norms[18], norms[19],
    norms[20], norms[21], norms[22], norms[23],
    norms[24], norms[25], norms[26], norms[27],
    norms[28], norms[29], norms[30], norms[31]);

  printf("\n");
}

// print normed EMG
void printNormedEMG(float *emg)
{
  printf("Normalized EMG:\n");
  printf("\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n",
    emg[0], emg[4], emg[8],  emg[12],
    emg[1], emg[5], emg[9],  emg[13],
    emg[2], emg[6], emg[10], emg[14],
    emg[3], emg[7], emg[11], emg[15]);

  printf("\n");
}

// print muscle activation
void printMuscleActivation(float *act)
{
  printf("Muscle Activation:\n");
  printf("\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n",
    act[0], act[4], act[8],  act[12],
    act[1], act[5], act[9],  act[13],
    act[2], act[6], act[10], act[14],
    act[3], act[7], act[11], act[15]);

  printf("\n");
}

//////////////////////////////////////////////////
// Print tracking data from Polhemus system
void printPolhemus(polhemus_pose_t *poses, int num_poses)
{
  int i;
  for(i = 0; i < num_poses; i++)
    printf("%d: %lf %lf %lf %lf %lf %lf\n", poses[i].station_id,
          poses[i].x, poses[i].y, poses[i].z,
          poses[i].roll, poses[i].pitch, poses[i].yaw);

  printf("\n");

  // NOTE ON POLHEMUS TRACKERS
    // finger 1: seems to be nothing
    // finger 2: seems to be nothing
    // arm: arm
    // head 2: head, but side to side/tilting are switched
}
