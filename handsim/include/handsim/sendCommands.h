// Header file to read in commands

#ifdef _WIN32
#include <windows.h>
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>
#include <haptix/comm/haptix.h>
#ifdef _WIN32
#include <windows.h>
#endif

// sendCommands.h
void sendCommand(int counter, hxRobotInfo *robotInfo, hxCommand *cmd);