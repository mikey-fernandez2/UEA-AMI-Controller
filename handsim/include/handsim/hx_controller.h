// Header file for hx_controller (both original and modified versions)
//
// Mikey Fernandez 04/14/2021

#ifndef HX_CONTROLLER_H
#define HX_CONTRLLER_H

#include <math.h>
#include <stdbool.h>
#include <signal.h>
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
#include "calculateCommands.h"
#include "polhemus_driver.h"
#include "printFunctions.h"

void sigHandler(int signo);

void pose_cb(double x, double y, double z, double roll, double pitch, double yaw);

#endif