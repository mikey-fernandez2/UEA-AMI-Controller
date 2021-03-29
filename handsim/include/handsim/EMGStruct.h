// Header file containing the Wifi EMG board unpacking struct
#include <stdint.h>

struct EMGData {
    uint32_t OS_time;
    uint16_t OS_tick;
    float rawEMG[16];
    unsigned int trigger : 1;
    unsigned int switch1 : 1;
    unsigned int switch2 : 1;
    unsigned int end;
};