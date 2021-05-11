// Header file containing the structs related to collecting EMG data from the WiFi board
//
// Mikey Fernandez 02/2021

#ifndef EMG_STRUCT_H
#define EMG_STRUCT_H

#include <string.h>
#include <stdint.h>

struct __attribute__((__packed__)) EMGData {
    float OS_time; // fields corresponding to data packet from EMG board
    float OS_tick;
    float rawEMG[16];
    uint32_t trigger;
    uint32_t switch1;
    uint32_t switch2;
    uint32_t end;
    float samplingFreq;

    int numElec;  // number of electrodes on SeongHo's EMG board - should be 16
    float tauA;   // activation time constant
    float tauD;   // deactivation time constant

    float bounds[32];    // first 16: maximum values, second 16: minimum values
    float normedEMG[16]; // array of normalized EMG values

    float deltas[16];    // first 8: maximum deltas, second 8: minimum deltas

    float muscleAct[16]; // array of muscle activation (through low pass muscle activation dynamics)
    float prevAct[16];   // array of previous muscle activation values
};

void normEMG(struct EMGData *emg);

float getNormedEMG(struct EMGData *emg, int i);

float getFilteredEMG(struct EMGData *emg, int i);

void muscleDynamics(struct EMGData *emg);

void getElectrodes(int motor, int *agonist, int *antagonist);

void getGains(int motor, float *gains, int numElec);

#endif