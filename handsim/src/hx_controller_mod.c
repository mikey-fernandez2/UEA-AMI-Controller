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

#include <math.h>
#include <stdbool.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <haptix/comm/haptix.h>
// #include "EMGStruct.h"
#include "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/handsim/EMGStruct.h"
// #include "calculateCommands.h"
#include "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/handsim/calculateCommands.h"
// #include "handsim/include/handsim/polhemus_driver.h"
#include "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/handsim/polhemus_driver.h"
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <unistd.h>
#include "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/handsim/printFunctions.h"

int running = 1;

//////////////////////////////////////////////////
void sigHandler(int signo)
{
  // Terminate the program.
  running = 0;
}

// All printing functions relegated to "printFunctions.c"

// Divide each raw EMG value by the normalization factor and save in struct
void normEMG(EMGData *emg, int numElec)
{
  int i = 0;

  for(i = 0; i < numElec; i++)
  {
    emg->normedEMG[i] = emg->rawEMG[i]/emg->MVC[i];
  }
}

//////////////////////////////////////////////////
// This main function requires two arguments
//    usingEMG is a boolean integer - 0 if not, 1 if using, EMG board
//    usingPolhemus is a boolean integer - 0 if not, 1 if using, Polhemus trackers
int main(int argc, char **argv)
{
  bool usingEMG;
  bool usingPolhemus;

  if (argc != 3)
  {
    fprintf(stderr, "Wrong number of arguments. Did you set usingEMG and usingPolhemus?\n");
    return -1;
  }
  else
  {
    usingEMG = strcmp("0", argv[1]) == 0 ? false : true;
    usingPolhemus = strcmp("0", argv[2]) == 0 ? false : true;
  }

  char *str;
  if (usingEMG && usingPolhemus)
  {
    str = "with EMG and Polhemus";
  }
  else if (usingEMG)
  {
    str = "with EMG";
  }
  else if (usingPolhemus)
  {
    str = "with Polhemus";
  }
  else
  {
    str = "without EMG or Polhemus";
  }
  printf("Starting arm controller %s\n", str);
   
  int i;
  int counter = 0;
  hxRobotInfo robotInfo;
  hxCommand cmd = {0};
  hxSensor sensor = {0};
  clock_t start, end, loopStart, loopEnd;

  // Polhemus settings
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

  printf("Robot checks passed.\n\n");

  // Print the robot information.
  printRobotInfo(&robotInfo);

  // Uncomment this block to start logging.
  // if (hxs_start_logging("/tmp/log/") != hxOK)
  //   printf("hxs_start_logging(): error.\n");

  ///////////////////////////////
  // Set up for receiving EMG data
  ssize_t n;

  int msgLen = 76; // 76 bytes - IHffffffffffffffffBBBB struct formatting
  char buffer[msgLen];
  EMGData *emg = malloc(sizeof(EMGData));
  char *EMGPipe = "/tmp/emg"; // Pipe for transmitting EMG data
  int fd1;                    // Pipe file descriptor

  int numElec = 16;
  int scaleFactorsLen = 4*numElec; // 64 bytes - ffffffffffffffff struct formatting
  char buffer2[scaleFactorsLen];
  char *scaleFactors = "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/scaleFactors.txt";
  int fd2;

  if (usingEMG)
  {
    printf("Trying to connect to EMG board...\n");

    mkfifo(EMGPipe, 0666); 

    printf("Successfully connected to EMG board.\n\n");

    printf("Trying to read EMG normalization factors...\n");

    fd2 = open(scaleFactors, O_RDONLY); 
    n = read(fd2, buffer2, scaleFactorsLen);
    close(fd2);
    if (n >= 0) 
    {
      memcpy(emg->MVC, buffer2, scaleFactorsLen); // copy EMG MVC norming factors into appropriate field of struct
      printEMGNorms(emg->MVC);
    }
    else
    {
      printf("read(): Receiving error.\n");
      return -1;
    }

    printf("Successfully read EMG norming factors.\n\n");
  }

  ///////////////////////////////
  start = clock(); end = clock();

  int steps = 0;

  // Send commands, read from sensors
  while (running)
  {
    loopStart = clock(); // for adjusting wait time
    // printf("Time running: %f sec\n", 1000*(double)(end - start)/CLOCKS_PER_SEC);

    // calculate next control command
    calculateCommands(&robotInfo, &cmd, &sensor, emg, usingEMG, counter);

    // Send the new joint command and receive the state update.
    if (hx_update(&cmd, &sensor) != hxOK)
    {
      printf("hx_update(): Request error.\n");
      continue;
    }
    // printf("Command %d sent\n", steps);

    // Debug output: Print the state.
    if (!(counter % 100))
    {
      // printCommand(&robotInfo, &cmd);
      printState(&robotInfo, &sensor); // printState() cannot be commented out or the limb won't move
    }
    if (++counter == 2000) // originally 10000
      counter = 0;

    ++steps;

    ////// Perform sensor reading below

    if (usingPolhemus)
    {
      polhemus_get_poses(conn, poses, &num_poses, 10);
      // printPolhemus(poses, num_poses); // print received Polhemus poses
    }

    if (usingEMG)
    {
      fd1 = open(EMGPipe, O_RDONLY); 
      n = read(fd1, buffer, msgLen);
      close(fd1);
      if (n >= 0) 
      {
        memcpy(emg, buffer, msgLen); // copy incoming data into the EMG data struct
        normEMG(emg, numElec);       // calculate and store normed EMG values
        printEMGData(emg);           // print EMG struct
      }
      else
      {
        printf("read(): Receiving error.\n");
        return -1;
      }
    }

    // This is the 'end' point - stop running the control loop
    if(running == 0)
    {
      printf("\nEnding movement\n");
      break;
    }

    loopEnd = clock();
    unsigned int sleeptime_us = 1000 - (int)(loopEnd - loopStart)*1e6/CLOCKS_PER_SEC; // adjustment made for how long running this loop takes

    usleep(sleeptime_us);
    end = clock();

    // printf("Control loop ran %i times.\n", steps);
  }

  ///////////////////////////////
  // Uncomment this block to stop logging.
  // if (hxs_stop_logging() != hxOK)
  //   printf("hxs_stop_logging(): error.\n");

  // Disconnect from the simulator / hardware
  if (hx_close() != hxOK)
  {
    printf("hx_close(): Request error.\n");
    return -1;
  }

  // free the memory used
  free(emg);

  printf("Exited successfully.\n");

  return 0;
}