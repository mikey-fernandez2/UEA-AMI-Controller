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
void muscleDynamics(struct EMGData *emg, float tauA, float tauD)
{
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
  int motorMap[23][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {6, 7}, {6, 7}, {8, 9}, {8, 9}, {8, 9}, {8, 9},
                                {10, 11}, {10, 11}, {10, 11}, {12, 13}, {12, 13}, {12, 13}, {12, 13}, {14, 15},
                                {14, 15}, {14, 15}, {14, 15}};

  *agonist = motorMap[motor][0];
  *antagonist = motorMap[motor][1];

  return;  
}
