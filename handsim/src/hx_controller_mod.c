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

#include "../include/handsim/hx_controller.h"

int running = 1;

//////////////////////////////////////////////////
void sigHandler(int signo)
{
  // Terminate the program.
  running = 0;
}

// All printing functions relegated to "printFunctions.c"

//////////////////////////////////////////////////
// This main function takes two arguments and an optional log file name
//    usingEMG is a boolean integer      - 0 if not, 1 if using, EMG board
//    usingPolhemus is a boolean integer - 0 if not, 1 if using, Polhemus trackers
//    logFile is the name of the log file to save information to (by default in /home/haptix-e15-463/haptix/haptix_controller/logs/)

int main(int argc, char **argv)
{
  bool usingEMG;
  bool usingPolhemus;
  bool logging;
  char *logFile;

  if (argc == 3 || argc == 4)
  {
    usingEMG = strcmp("0", argv[1]) == 0 ? false : true;
    usingPolhemus = strcmp("0", argv[2]) == 0 ? false : true;
    logging = argc == 4 ? true : false;
    
    if (argc == 4)
    {
      logFile = argv[3];
    }
  }
  else
  {
    fprintf(stderr, "Wrong number of arguments.\nDid you set usingEMG and usingPolhemus? Did you name a log file?\n");
    return -1;
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
  struct timeval startT, endT;

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
  printf("Performing robot checks...\n");
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

  // Print the robot information.
  printRobotInfo(&robotInfo);
  
  printf("Robot checks passed.\n\n");

  // Uncomment this block to start logging.
  // if (hxs_start_logging("/tmp/log/") != hxOK)
  //   printf("hxs_start_logging(): error.\n");

  ///////////////////////////////
  // Set up for receiving EMG data
  ssize_t n;

  int msgLen = 92; // 92 bytes - ffffffffffffffffffIIIIf struct formatting
  char buffer[msgLen];
  struct EMGData *emg = malloc(sizeof(struct EMGData));
  memset(emg, 0, sizeof(struct EMGData)); // zero out EMG struct fields
  char *EMGPipe = "/tmp/emg"; // Pipe for receiving EMG data
  int fd1;                    // Pipe file descriptor

  int numElec = 16;
  emg->numElec = numElec;
  int scaleFactorsLen = 8*numElec; // 128 bytes - ffffffffffffffffffffffffffffffff struct formatting
  char buffer2[scaleFactorsLen];
  char *scaleFactors = "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/scaleFactors.txt";
  int fd2;

  int numPairs = numElec/2;
  int deltasLen = 8*numPairs; // 64 bytes - ffffffffffffffff struct formatting
  char buffer3[deltasLen];
  char *deltasPath = "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/deltas.txt";
  int fd3;

  float tauA = 0.05; // 50 ms activation time constant
  float tauD = 0.10; // 100 ms deactivation time constant

  struct secOrd *dynamics = malloc(sizeof(struct secOrd));
  memset(dynamics, 0, sizeof(dynamics)); // zero out struct
  dynamics->freq_n = 4; // frequency (Hz) of critically damped limb movement

  if (usingEMG)
  {
    emg->tauA = tauA;
    emg->tauD = tauD;

    printf("Trying to connect to EMG board...\n");

    mkfifo(EMGPipe, 0666); 

    printf("Successfully connected to EMG board.\n\n");

    printf("Reading EMG normalization factors...\n");

    fd2 = open(scaleFactors, O_RDONLY); 
    n = read(fd2, buffer2, scaleFactorsLen);
    close(fd2);
    if (n >= 0) 
    {
      memcpy(emg->bounds, buffer2, scaleFactorsLen); // copy EMG normalization bounds into appropriate field of struct
      // printEMGNorms(emg->bounds);
    }
    else
    {
      printf("read(): Error reading EMG norms.\n");
      return -1;
    }

    printf("Successfully read EMG norming factors.\n\n");

    printf("Reading EMG delta bounds...\n");

    fd3 = open(deltasPath, O_RDONLY); 
    n = read(fd3, buffer3, deltasLen);
    close(fd3);
    if (n >= 0) 
    {
      memcpy(emg->deltas, buffer3, deltasLen); // copy EMG normalization bounds into appropriate field of struct
    }
    else
    {
      printf("read(): Error reading EMG delta bounds.\n");
      return -1;
    }

    printf("Successfully read EMG delta bounds.\n\n");
  }

  ////////////////////////////////
  // start logging
  char logPath[1000] = "/home/haptix-e15-463/haptix/haptix_controller/logs/";

  if (logging)
  {
    // strcat(logPath, logFile); strcat(logPath, ".txt"); // get full path to log file
    strcat(logPath, logFile); strcat(logPath, ".csv"); // get full path to log file in .csv format

    if (!startLogging(logPath, usingEMG, usingPolhemus, &robotInfo, emg, dynamics, num_poses))
    {
      printf("startLogging(): error.\n");
      return -1;
    }
    printf("Logging to %s.\n", logPath);
  }

  printf("All tests passed. Beginning movement.\n");
  ///////////////////////////////
  start = clock(); end = clock(); // for timing control loop

  int steps = 0;
  int waitTime;

  gettimeofday(&startT, NULL);
  long double first = startT.tv_sec + 1e-6*startT.tv_usec;
  // Send commands, read from sensors
  while (running)
  {
    // This is the 'end' point - stop running the control loop
    if (running == 0)
    {
      printf("\nEnding movement.\n");
      break;
    }

    loopStart = clock(); // for adjusting wait time

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
        memcpy(emg, buffer, msgLen); // copy incoming EMG data into EMG struct
        normEMG(emg);                // calculate and store normed EMG values, updating 'prevEMG' field
        muscleDynamics(emg);         // use low pass muscle activation dynamics with tauA and tauD
        // printEMGData(emg);           // print EMG struct
        // printMuscleActivation(emg->muscleAct);
        // printNormedEMG(emg->normedEMG);
      }
      else
      {
        printf("read(): Error reading EMG data.\n");
        return -1;
      }

      ++steps;
    }

    // calculate next control command
    calculateCommands(&robotInfo, &cmd, &sensor, emg, dynamics, usingEMG, counter);
    // printCommandMotor(&robotInfo, &cmd, 2);

    // Send the new joint command and receive the state update.
    if (hx_update(&cmd, &sensor) != hxOK)
    {
      printf("hx_update(): Request error.\n");
      continue;
    }

    // Debug output: Print the state.
    if (!(counter % 20))
    {
      // printCommand(&robotInfo, &cmd);
      printState(&robotInfo, &sensor); // printState() cannot be commented out or the limb won't move [however, function was modified to do nothing]
     
      if (logging)
      {
        gettimeofday(&endT);
        long double sec = endT.tv_sec + 1e-6*endT.tv_usec;

        if (!addLog(logPath, usingEMG, usingPolhemus, sec - first, &robotInfo, &cmd, &sensor, poses, num_poses, emg))
        {
          printf("addLog(): error.\n");
          return -1;
        }
      }
    }

    if (++counter == 10000) // originally 10000
    {
      counter = 0;
    }

    loopEnd = clock();

    waitTime = usingEMG ? 1e6/emg->samplingFreq : 1000; // wait longer if using EMG
    unsigned int sleeptime_us = waitTime - (int)((loopEnd - loopStart)*1e6/CLOCKS_PER_SEC); // adjustment made for how long running this loop takes
    if (sleeptime_us <= 0)
    {
      sleeptime_us = waitTime; // if loop took too long to run, just wait the default time
    }

    usleep(sleeptime_us);
    end = clock();
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

  // stop the log
  if (logging)
  {
    if (!endLog(logPath))
    {
      printf("endLog(): error. \n");
      return -1;
    }
  }

  // free the memory used
  free(emg);
  free(dynamics);

  printf("Exited successfully.\n");

  return 0;
}