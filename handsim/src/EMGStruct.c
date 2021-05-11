// Contains functions to manipulate the EMGData struct
//
// Mikey Fernandez 04/14/2021

#include "../include/handsim/EMGStruct.h"

// Normalize raw EMG signals and save in struct
void normEMG(struct EMGData *emg)
{
  int i;
  float max, min;
  float normed;
  for (i = 0; i < emg->numElec; i++)
  {
    max = emg->bounds[i];
    min = emg->bounds[emg->numElec + i];

    normed = (emg->rawEMG[i] - min)/max;

    if (normed < 0)
    {
      normed = 0;
    }
    else if (normed > 1)
    {
      normed = 1;
    }

    emg->normedEMG[i] = normed;
  }
}

// extract normed EMG values
float getNormedEMG(struct EMGData *emg, int i)
{
  return emg->normedEMG[i];
}

// extract muscle activation filtered EMG values
float getFilteredEMG(struct EMGData *emg, int i)
{
  return emg->muscleAct[i];
}

// implement low pass filter corresponding to muscle activation dynamics
void muscleDynamics(struct EMGData *emg)
{
  float tauA = emg->tauA;
  float tauD = emg->tauD;

  float b = tauA/tauD; // ratio of time constants
  float u;             // normed EMG
  float prevA;         // previous activation
  float fs = emg->samplingFreq; // sampling frequency
  float Ts = 1/fs;              // sampling time

  int i;
  for (i = 0; i < emg->numElec; i++)
  {
    u = emg->normedEMG[i];
    emg->prevAct[i] = emg->muscleAct[i]; // store previous muscle activation value
    prevA = emg->prevAct[i];

    emg->muscleAct[i] = (u/tauA + prevA/Ts)/(1/Ts + (b + (1 - b)*u)/tauA);
  }
}

// get the electrodes corresponding to the agonist and antagonist muscles for given motor/joint
void getElectrodes(int motor, int *agonist, int *antagonist)
{
  // motorMap[i][0] is the electrode corresponding to the agonist muscle for motor i
  // motorMap[i][1] is the electrode corresponding to the antagonist muscle for motor i
  int motorMap[14][2] = {{0, 1},                         // elbow
                         {2, 3}, {4, 5}, {6, 7},         // wristx, wristy, wristz
                         {8, 9}, {14, 15}, {14, 15}, {14, 15}, // thumb0, thumb1, thumb2, thumb3
                         {10, 11}, {10, 11},             // index0, index1
                         {12, 13},                       // middle1
                         {12, 13},                       // ring1
                         {12, 13}, {12, 13}};            // pinky0, pinky1

  *agonist = motorMap[motor][0];
  *antagonist = motorMap[motor][1];
}

// get the gains for each electrode to control a given movement
//   the gains come from a 2D array of gains, which is (numDoF) x (numElectrodes)
void getGains(int motor, float *gains, int numElec)
{                // electrode: 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15    gains corresponding to
  float gainMatrix[14][16] = {{0.354, 0.0097, 1.0913, -0.8021, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // motor0:  elbow 
                              {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // motor1:  wrist_y 
                              {0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // motor2:  wrist_x
                              {0.9321, -0.7007, 0.0517, 0.0203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // motor3:  wrist_z
                              {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},  // motor4:  thumb_0
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},  // motor5:  thumb_1
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},  // motor6:  thumb_2
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},  // motor7:  thumb_3
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0},  // motor8:  index_0
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0},  // motor9:  index_1
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},  // motor10: middle_1
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},  // motor11: ring_1
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},  // motor12: pinky_0
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0}}; // motor13: pinky_1

  int i;
  for (i = 0; i < numElec; i++)
  {
    *(gains + i) = gainMatrix[motor][i];
  }
}