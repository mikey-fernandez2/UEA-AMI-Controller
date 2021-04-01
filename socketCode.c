// Written to remove dead code from controller
// Contains code needed to run socket between EMG board and controller
// Contains code for running on Windows

//////////////////////////////////////////////////
// Windows: include statements
#ifdef _WIN32
#include <windows.h>
#define _USE_MATH_DEFINES
#endif

#ifdef _WIN32
#include <windows.h>
#endif

//////////////////////////////////////////////////
// Windows: sleep/timing lines
#ifdef _WIN32
    Sleep((DWORD)(sleeptime_us / 1e3));
#else
#endif

//////////////////////////////////////////////////
//////////////////////////////////////////////////

//////////////////////////////////////////////////
// include statements
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>

//////////////////////////////////////////////////
// creating and connecting socket
#define gIP "192.168.50.192"
#define gPORT 8899
int EMGSock;

struct addrinfo hints = {0};
struct addrinfo *servInfoList;
struct addrinfo *selfInfoList;
hints.ai_family = AF_INET;
hints.ai_socktype = SOCK_DGRAM;
hints.ai_protocol = 0;
hints.ai_flags = AI_PASSIVE;
hints.ai_canonname = NULL;
hints.ai_addr = NULL;
hints.ai_next = NULL;

EMGSock = socket(AF_INET, SOCK_DGRAM, 0);
// printf("Socket fileID: %d\n", EMGSock);
if (EMGSock < 0)
{
    printf("socket(): Socket creation error\n");
    return -1;
}

int serverInfo = getaddrinfo(gIP, "8899", &hints, &servInfoList);
int selfInfo = getaddrinfo(NULL, "50000", &hints, &selfInfoList);
if (serverInfo < 0 || selfInfo < 0)
{
    printf("getaddrinfo(): Error");
    return -1;
}
if (serverInfo == 0)
{
    struct addrinfo *rp;
    for (rp = selfInfoList; rp != NULL; rp = rp->ai_next)
    {

    EMGSock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (EMGSock < 0)
        continue;

    if (bind(EMGSock, rp->ai_addr, rp->ai_addrlen) != -1)
    {
        // struct sockaddr_in *addr_in = (struct sockaddr_in *)rp;
        // printf("Found my address: %s\n", inet_ntoa(addr_in->sin_addr));
        // printf("Found my port: %d\n", addr_in->sin_port);
        // char s[INET6_ADDRSTRLEN];
        // printf("Address: %s\n",inet_ntop(rp->ai_family, &(((struct sockaddr_in*)(struct sockaddr *)rp->ai_addr)->sin_addr), s, sizeof(s)));
        struct sockaddr_in addr;
        socklen_t len;
        len = sizeof(addr);
        getpeername(EMGSock, (struct sockaddr *)&addr, &len);
        printf("Found server address: (%s:%d)\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
        break;
    }
    close(EMGSock);
    }
    if (rp == NULL)
    {
    printf("None of the addresses can be connected to\n");
    return -1;
    }
}
else
{
    printf("Can't find server at given address\n");
    return -1; Debug output: 
}

//////////////////////////////////////////////////
// receive EMG data through socket
// Receive new EMG data
printf("Receiving new data...\n");
n = recv(EMGSock, (char *)buffer, msgLen, 0);
printf("Bytes received: %d\n", (int)n);
if (n >= 0) 
{
memcpy(emg, buffer, msgLen); // copy incoming data into the EMG data struct
// printf(Electrode 0: %f\n", emg->rawEMG[0]); // to verfiy incoming data is correct
printEMGData(emg);
}
else
{
printf("recv(): Receiving error.\n");
return -1;
}