// Header file for functions to log data from HAPTIX Gazebo and commands
//
// Mikey Fernandes 04/23/2021

#ifndef LOGGING_H
#define LOGGING_H

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <haptix/comm/haptix.h>
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <unistd.h>
#include "EMGStruct.h"
#include "polhemus_driver.h"
#include "calculateCommands.h"

int startLogging(char *logPath, bool usingEMG, bool usingPolhemus, hxRobotInfo *robotInfo, struct EMGData *emg, struct secOrd *dynamics);

int addLog(char *logPath, bool usingEMG, bool usingPolhemus, long double runTime, hxRobotInfo *robotInfo, hxCommand *cmd, hxSensor *sensor, polhemus_pose_t *poses, int num_poses, struct EMGData *emg);

int endLog(char *logPath);

#endif