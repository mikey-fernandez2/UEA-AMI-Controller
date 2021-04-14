// Contains functions to manipulate the EMGData struct
//
// Mikey Fernandez 04/14/2021

#include "../include/handsim/EMGStruct.h"

// Divide each raw EMG value by the normalization factor and save in struct
void normEMG(struct EMGData *emg, int numElec)
{
  int i = 0;
  for(i = 0; i < numElec; i++)
  {
    emg->normedEMG[i] = emg->rawEMG[i]/emg->MVC[i];
  }
}

// extract normed EMG values
float getNormedEMG(struct EMGData *emg, int i)
{
  return emg->normedEMG[i];
}