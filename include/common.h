#ifndef COMMON_H
#define COMMON_H

#include "Arduino.h"
#include "io.h"
#include "checksums.h"
#include "canMessages.h"
#include "hondaSerial.h"

void updateStatus(struct Status *status);
bool timeSince(uint32_t *timer, uint32_t compareTime, bool autoReset);

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
int16_t clip(int16_t x, int16_t lo, int16_t hi);

#define steerMax  239
#define steerThreshold  30
#define steerDeltaUp  7
#define steerDeltaDown  100
#define steerDriverAllowance  25
#define steerDriverMultiplier  18
#define steerDriverFactor  1

struct Error {
    bool inError;
    bool lateMsg;
    bool lateCanMsg;
    uint8_t invalidCounterCount;
    uint8_t invalidChecksumCount;
};

struct Can {
    bool     txFirmwareVersion;
    bool     msgDetected;        // CAN message detection
    bool     lkasRequest;       // LKAS allowed OP
    uint8_t  opCounter;
    uint16_t steerTorque;       // Torque request value (raw)
    uint8_t  steerB0;           // Byte 0 (little steer byte)
    uint8_t  steerB1;           // Byte 1 (big steer byte)
};

struct Timer {
    bool steeringControlTimeFlag;
    uint32_t steeringControlTime_LKAS;
    uint32_t steeringControlTime_EPS;
    uint32_t can100HzTimer;
    uint32_t ledTimer;
    uint32_t versionTimer;
};

struct Status {
    bool lkasAllowed;
    bool msgSteerRequest;   // Detection of a 
    bool steerBlended;

    uint16_t steerTorqueLast;
    uint16_t driverAppliedSteer;
    uint8_t epsData[5];
    uint8_t lkasData[5];
    uint8_t counter;
    uint8_t sendFrameDelay; // Stock system applies 5 frames of applysteer = 0
    
    struct Error error;
    struct Can can;
    struct Timer timers;
};

 // Build Options

#endif