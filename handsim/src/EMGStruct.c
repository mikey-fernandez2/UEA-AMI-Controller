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
  // motorMap[i][o] is the electrode corresponding to the agonist muscle for motor i
  // motorMap[i][1] is the electrode corresponding to the antagonist muscle for motor i
  int motorMap[14][2] = {{0, 1},                         // elbow
                         {8, 8}, {8, 8}, {8, 8},         // wristx, wristy, wristz
                         {3, 4}, {3, 4}, {3, 4}, {3, 4}, // thumb0, thumb1, thumb2, thumb3
                         {3, 4}, {3, 4},             // index0, index1
                         {3, 4},                       // middle1
                         {3, 4},                       // ring1
                         {3, 4}, {3, 4}};            // pinky0, pinky1

  *agonist = motorMap[motor][0];
  *antagonist = motorMap[motor][1];

  return;  
}