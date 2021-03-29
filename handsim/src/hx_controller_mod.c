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

#ifdef _WIN32
#include <windows.h>
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <haptix/comm/haptix.h>
#ifdef _WIN32
#include <windows.h>
#endif
// #include "EMGStruct.h"
#include "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/handsim/EMGStruct.h"
// #include "sendCommands.h"
#include "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/handsim/sendCommands.h"
// #include <sys/socket.h>
// #include <sys/types.h>
// #include <netdb.h>
// #include <arpa/inet.h>
// #include <netinet/in.h>
// #include "handsim/include/handsim/polhemus_driver.h"
#include "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/handsim/polhemus_driver.h"
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <unistd.h>

// #define gIP "192.168.50.192"
// #define gPORT 8899

int running = 1;
int usingEMG;
int usingPolhemus;

//////////////////////////////////////////////////
void sigHandler(int signo)
{
  // Terminate the program.
  running = 0;
}

//////////////////////////////////////////////////
void printState(const hxRobotInfo *_robotInfo, const hxSensor *_sensor)
{
  return;

  // int i;

  // printf("\tTime: %d.%09d\n",
  //   _sensor->time_stamp.sec, _sensor->time_stamp.nsec);

  // printf("\tMotors:\n");
  // for (i = 0; i < _robotInfo->motor_count; ++i)
  // {
  //   printf("\t\tMotor %d\n", i);
  //   printf("\t\t\tPosition: %f rads\n", _sensor->motor_pos[i]);
  //   printf("\t\t\tVelocity: %f rads/sec\n", _sensor->motor_vel[i]);
  //   printf("\t\t\tTorque: %f N*m\n" , _sensor->motor_torque[i]);
  // }

  // printf("\tJoints:\n");
  // for (i = 0; i < _robotInfo->joint_count; ++i)
  // {
  //   printf("\t\tJoint %d\n", i);
  //   printf("\t\t\tPosition: %f rads\n", _sensor->joint_pos[i]);
  //   printf("\t\t\tVelocity: %f rads/sec\n", _sensor->joint_vel[i]);
  // }

  // printf("\tContact sensors:\n");
  // for (i = 0; i < _robotInfo->contact_sensor_count; ++i)
  // {
  //   printf("\t\t# %d\n", i);
  //   printf("\t\t\tvalue: %f N.\n", _sensor->contact[i]);
  // }

  // printf("\tIMUs:\n");
  // for (i = 0; i < _robotInfo->imu_count; ++i)
  // {
  //   printf("\t\t# %d\n", i);
  //   printf("\t\t\tLinear acceleration: (%f, %f, %f) m/sec2\n",
  //     _sensor->imu_linear_acc[i][0], _sensor->imu_linear_acc[i][1],
  //     _sensor->imu_linear_acc[i][2]);
  //   printf("\t\t\tAngular velocity: (%f, %f, %f) rads/sec\n",
  //     _sensor->imu_angular_vel[i][0], _sensor->imu_angular_vel[i][1],
  //     _sensor->imu_angular_vel[i][2]);
  // }

  // printf("\n");
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
    printf("\t\t Min: %f rads.\n", _robotInfo->joint_limit[i][0]);
    printf("\t\t Max: %f rads.\n", _robotInfo->joint_limit[i][1]);
  }

  printf("\n");
}

//////////////////////////////////////////////////
void printCommand(const hxRobotInfo *_robotInfo, const hxCommand *_cmd)
{
  int i;

  printf("Command received:\n");

  printf("\tref_pos_enabled: %d\n", _cmd->ref_pos_enabled);
  printf("\tref_vel_enabled: %d\n", _cmd->ref_vel_enabled);
  printf("\tref_vel_max_enabled: %d\n", _cmd->ref_vel_max_enabled);
  printf("\tgain_pos_enabled: %d\n", _cmd->gain_pos_enabled);
  printf("\tgain_vel_enabled: %d\n", _cmd->gain_vel_enabled);

  printf("\n\tMotors:\n");
  for (i = 0; i < _robotInfo->motor_count; ++i)
  {
    printf("\t\tMotor %d\n", i);
    printf("\t\t\tref_pos: %f rads\n", _cmd->ref_pos[i]);
    printf("\t\t\tref_vel: %f rads/sec\n", _cmd->ref_vel[i]);
    printf("\t\t\tref_vel_max: %f rads/sec\n", _cmd->ref_vel_max[i]);
    printf("\t\t\tgain_pos: %f Nm/rad\n", _cmd->gain_pos[i]);
    printf("\t\t\tgain_vel: %f Nm*sec/rad\n", _cmd->gain_vel[i]);
  }

  printf("\n");
}

//////////////////////////////////////////////////
void printEMGData(const struct EMGData *emg)
{
  time_t raw_time = (time_t)emg->OS_time;
  struct tm *timeinfo = localtime(&raw_time);
  
  struct tm ts;
  ts = *localtime(&raw_time);
  char timebuf[80];
  strftime(timebuf, sizeof(timebuf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);

  printf("EMG data received:\n");
  // printf("\tLocal time: %s", asctime(timeinfo));
  // printf("\tLocal time: %s\n", timebuf);
  // printf("\tRaw time: %d\n", emg->OS_time);
  printf("\tRunning time: %d ms\n", emg->OS_tick);
  printf("\tEMG:\n");
  printf("\t\t%10.2f\t%10.2f\t%10.2f\t%10.2f\n\t\t%10.2f\t%10.2f\t%10.2f\t%10.2f\n\t\t%10.2f\t%10.2f\t%10.2f\t%10.2f\n\t\t%10.2f\t%10.2f\t%10.2f\t%10.2f\n",
    emg->rawEMG[0], emg->rawEMG[4], emg->rawEMG[8], emg->rawEMG[12],
    emg->rawEMG[1], emg->rawEMG[5], emg->rawEMG[9], emg->rawEMG[13],
    emg->rawEMG[2], emg->rawEMG[6], emg->rawEMG[10], emg->rawEMG[14],
    emg->rawEMG[3], emg->rawEMG[7], emg->rawEMG[11], emg->rawEMG[15]);
  printf("\tTrigger: %d\n", emg->trigger);
  printf("\tSwitch1: %d\n", emg->switch1);
  printf("\tSwitch2: %d\n", emg->switch2);

  printf("\n");
}

void printPolhemus(polhemus_pose_t *poses, int num_poses)
{
  int i;
  for(i = 0; i < num_poses; i++)
    printf("%d: %lf %lf %lf %lf %lf %lf\n", poses[i].station_id,
          poses[i].x, poses[i].y, poses[i].z,
          poses[i].roll, poses[i].pitch, poses[i].yaw);

  printf("\n");

  /* NOTE POLHEMUS TRACKERS
  finger 1: seems to be nothing
  finger 2: seems to be nothing
  arm: arm
  head 2: head, but side to side/tilting are switched
  */
}

//////////////////////////////////////////////////
// This main function requires two arguments
//    usingEMG is a boolean integer - 0 if not, 1 if using, EMG board
//    usingPolhemus is a boolean integer - 0 if not, 1 if using, Polhemus trackers

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    fprintf(stderr, "Wrong number of arguments. Did you set usingEMG and usingPolhemus?\n");
    return -1;
  }
  else
  {
    usingEMG = strcmp("0", argv[1]) == 0 ? 0 : 1;
    usingPolhemus = strcmp("0", argv[2]) == 0 ? 0 : 1;
  }


  int i;
  int counter = 0;
  hxRobotInfo robotInfo;
  hxCommand cmd = {0};
  hxSensor sensor = {0};
  clock_t start, end, loopStart, loopEnd;
  char *myfifo = "/tmp/emg";
  int fd1;

  // reading from Polhemus
  polhemus_conn_t* conn;
  double x, y, z, roll, pitch, yaw; // these hold the pose from the sensors
  int num_poses = 4;                // have 4 sensors
  polhemus_pose_t poses[num_poses];

  ///////////////////////////////
  if (usingPolhemus)
  {
    if(!(conn = polhemus_connect_usb(LIBERTY_HS_VENDOR_ID,
                                     LIBERTY_HS_PRODUCT_ID,
                                     LIBERTY_HS_WRITE_ENDPOINT,
                                     LIBERTY_HS_READ_ENDPOINT)))
    {
      fprintf(stderr, "Failed to connect to Polhemus\n");
      return -1;
    }

    if(polhemus_init_comm(conn, 100))
    {
      fprintf(stderr, "Failed to initialize comms to Polhemus\n");
      return -1;
    }
  }
  ///////////////////////////////

  printf("\n\nInitializing...\n");

  // Capture SIGINT signal.
  if (signal(SIGINT, sigHandler) == SIG_ERR)
    printf("Error catching SIGINT\n");

  // Capture SIGTERM signal.
  if (signal(SIGTERM, sigHandler) == SIG_ERR)
    printf("Error catching SIGTERM\n");

  // Connect to the simulator / hardware
  if (hx_connect(NULL, 0) != hxOK)
  {
    printf("hx_connect(): Request error.\n");
    return -1;
  }

  // Requesting robot information.
  if (hx_robot_info(&robotInfo) != hxOK)
  {
    printf("hx_getrobotinfo(): Request error.\n");
    return -1;
  }

  printf("Robot connection checks passed.\n\n");

  // Print the robot information.
  printRobotInfo(&robotInfo);

  // Uncomment this block to start logging.
  // if (hxs_start_logging("/tmp/log/") != hxOK)
  //   printf("hxs_start_logging(): error.\n");

  ///////////////////////////////
  // Set up sockets for receiving EMG data
  int msgLen = 76;
  char buffer[msgLen];
  struct EMGData *emg = malloc(sizeof(struct EMGData));
  ssize_t n;
  // int EMGSock;

  if (usingEMG)
  {
    printf("\nTrying to connect to EMG board...\n");

    // struct addrinfo hints = {0};
    // struct addrinfo *servInfoList;
    // struct addrinfo *selfInfoList;
    // hints.ai_family = AF_INET;
    // hints.ai_socktype = SOCK_DGRAM;
    // hints.ai_protocol = 0;
    // hints.ai_flags = AI_PASSIVE;
    // hints.ai_canonname = NULL;
    // hints.ai_addr = NULL;
    // hints.ai_next = NULL;

    // EMGSock = socket(AF_INET, SOCK_DGRAM, 0);
    // // printf("Socket fileID: %d\n", EMGSock);
    // if (EMGSock < 0)
    // {
    //   printf("socket(): Socket creation error\n");
    //   return -1;
    // }

    // int serverInfo = getaddrinfo(gIP, "8899", &hints, &servInfoList);
    // int selfInfo = getaddrinfo(NULL, "50000", &hints, &selfInfoList);
    // if (serverInfo < 0 || selfInfo < 0)
    // {
    //   printf("getaddrinfo(): Error");
    //   return -1;
    // }
    // if (serverInfo == 0)
    // {
    //   struct addrinfo *rp;
    //   for (rp = selfInfoList; rp != NULL; rp = rp->ai_next)
    //   {

    //     EMGSock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    //     if (EMGSock < 0)
    //       continue;

    //     if (bind(EMGSock, rp->ai_addr, rp->ai_addrlen) != -1)
    //     {
    //       // struct sockaddr_in *addr_in = (struct sockaddr_in *)rp;
    //       // printf("Found my address: %s\n", inet_ntoa(addr_in->sin_addr));
    //       // printf("Found my port: %d\n", addr_in->sin_port);
    //       // char s[INET6_ADDRSTRLEN];
    //       // printf("Address: %s\n",inet_ntop(rp->ai_family, &(((struct sockaddr_in*)(struct sockaddr *)rp->ai_addr)->sin_addr), s, sizeof(s)));
    //       struct sockaddr_in addr;
    //       socklen_t len;
    //       len = sizeof(addr);
    //       getpeername(EMGSock, (struct sockaddr *)&addr, &len);
    //       printf("Found server address: (%s:%d)\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
    //       break;
    //     }
    //     close(EMGSock);
    //   }
    //   if (rp == NULL)
    //   {
    //     printf("None of the addresses can be connected to\n");
    //     return -1;
    //   }
    // }
    // else
    // {
    //   printf("Can't find server at given address\n");
    //   return -1; Debug output: 
    // }

    mkfifo(myfifo, 0666); 

    printf("Successfully connected to EMG board.\n\n");
  }

  float wrist_flex, wrist_extend, wrist_net;
  float wrist_vel = 0;

  ///////////////////////////////
  start = clock(); end = clock(); // TODO: Figure out some timing stuff

  int steps = 0;

  // Send commands at ~100Hz.
  while (running)
  {

    loopStart = clock(); // for adjusting wait time
    // printf("Time running: %f sec\n", 1000*(double)(end - start)/CLOCKS_PER_SEC);

    if (usingEMG)
    {
      for (i = 0; i < robotInfo.motor_count; ++i)
      {
        // Set the desired velocity of this motor
        if (i == 2)
          cmd.ref_pos[i] = -wrist_vel;
        else
          cmd.ref_pos[i] = 0.0;
        // cmd.ref_pos[i] = 0.0;
        // cmd.ref_vel_max[i] = 5.0;
        // We could set a desired controller position gain
        // cmd.gain_pos[i] = 1.0;
        // We could set a desired controller velocity gain
        // cmd.gain_vel[i] = 1.0;
      }
      cmd.ref_pos_enabled = 1;
      cmd.ref_vel_enabled = 0;
      cmd.ref_vel_max_enabled = 0;
      cmd.gain_pos_enabled = 0;
      cmd.gain_vel_enabled = 0;
    }
    else
    {
      // Create a new command based on a sinusoidal wave.
      for (i = 0; i < robotInfo.motor_count; ++i)
      {
        // Set the desired position of this motor
        cmd.ref_vel[i] = (float)(20 * 0.5 *
          sin(0.05 * 2.0 * M_PI * counter * 0.01));
        // We could set a desired maximum velocity
        // cmd.ref_vel[i] = 10.0;
        cmd.ref_vel_max[i] = 200.0;
        // We could set a desired controller position gain
        // cmd.gain_pos[i] = 1.0;
        // We could set a desired controller velocity gain
        cmd.gain_vel[i] = 1000.0;
      }
      // Indicate that the positions we set should be used.
      cmd.ref_pos_enabled = 0;
      // We're not setting it, so indicate that ref_vel should be ignored.
      cmd.ref_vel_enabled = 1;
      // We're not setting it, so indicate that ref_vel_max should be ignored.
      cmd.ref_vel_max_enabled = 1;
      // We're not setting it, so indicate that gain_pos should be ignored.
      cmd.gain_pos_enabled = 0;
      // We're not setting it, so indicate that gain_vel should be ignored.
      cmd.gain_vel_enabled = 1;
    }

    // sendCommand(counter, &robotInfo, &cmd);

    // Send the new joint command and receive the state update.
    if (hx_update(&cmd, &sensor) != hxOK)
    {
      printf("hx_update(): Request error.\n");
      continue;
    }
    // printf("Command %d sent\n", steps);

    // Debug output: Print the state.
    // printState() cannot be commented out or the limb won't move
    if (!(counter % 100))
      printCommand(&robotInfo, &cmd);
      printState(&robotInfo, &sensor);
    if (++counter == 10000)
      counter = 0;

    ++steps;

    // Here is where you would do your other work, such as reading from EMG
    // sensors, decoding that data, computing your next control command,
    // etc.  In this example, we're just sleeping for 10ms.
    //
    // You might also want to sleep in your code, because there's a maximum
    // rate at which the limb can process new commands and produce new
    // sensor readings.  Depending on how long your computation takes, you
    // might want to wait here until it's time to send a new command.  Or
    // you might want to run as fast as possible, computing and sending new
    // commands constantly (but knowing that not all of them will be
    // executed by the limb).

    if (usingPolhemus)
    {
      // print out Polhemus state update
      polhemus_get_poses(conn, poses, &num_poses, 10);
      //printf("Received %d poses\n", num_poses);
      printPolhemus(poses, num_poses);
    }

    if (usingEMG)
    {
      // Receive new EMG data
      // printf("Receiving new data...\n");
      // n = recv(EMGSock, (char *)buffer, msgLen, 0);
      // printf("Bytes received: %d\n", (int)n);
      // if (n >= 0) 
      // {
      //   memcpy(emg, buffer, msgLen); // copy incoming data into the EMG data struct
      //   // printf(Electrode 0: %f\n", emg->rawEMG[0]); // to verfiy incoming data is correct
      //   printEMGData(emg);
      // }
      // else
      // {
      //   printf("recv(): Receiving error.\n");
      //   return -1;
      // }

      fd1 = open(myfifo, O_RDONLY); 
      n = read(fd1, buffer, msgLen);
      close(fd1);
      if (n >= 0) 
      {
        memcpy(emg, buffer, msgLen); // copy incoming data into the EMG data struct
        // printEMGData(emg);
      }
      else
      {
        printf("read(): Receiving error.\n");
        return -1;
      }

      // printf("Calculating new wrist command\n");
      wrist_flex = emg->rawEMG[7];
      wrist_extend = emg->rawEMG[6];
      wrist_net = wrist_extend - wrist_flex;
      wrist_vel = 2.0*wrist_net;
      printf("New wrist command: %f\n", wrist_vel);
    }

    if(running == 0)
    {
      printf("Stopping movement\n");
      break;
    }

    loopEnd = clock();
    unsigned int sleeptime_us = 10000 - (int)(loopEnd - loopStart)*1e6/CLOCKS_PER_SEC; // adjustment made for how long running this loop takes

#ifdef _WIN32
    Sleep((DWORD)(sleeptime_us / 1e3));
#else
    usleep(sleeptime_us);
    end = clock();
#endif
  }

  ///////////////////////////////
  printf("Movement completed.\n");

  // Uncomment this block to stop logging.
  // if (hxs_stop_logging() != hxOK)
  //   printf("hxs_stop_logging(): error.\n");

  // Disconnect from the simulator / hardware
  if (hx_close() != hxOK)
  {
    printf("hx_close(): Request error.\n");
    return -1;
  }

  if (usingEMG)
  {
    // close(EMGSock);
  }

  free(emg); // free the memory allocated for this struct

  return 0;
}