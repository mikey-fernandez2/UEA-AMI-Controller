// Header file containing the structs related to collecting EMG data from the WiFi board
#include <stdint.h>

struct EMGData {
    uint32_t OS_time;
    uint16_t OS_tick;
    float rawEMG[16];
    unsigned int trigger : 1;
    unsigned int switch1 : 1;
    unsigned int switch2 : 1;
    unsigned int end;
    float MVC[16];
    float normedEMG[16];
};

typedef struct EMGData EMGData;