#pragma once

#include <Arduino.h>
#include "event_timer.h"

class Robot
{
public:
    enum ROBOT_STATE {ROBOT_IDLE, ROBOT_PATROLLING, ROBOT_APPROACHING, ROBOT_DETERRING, ROBOT_STOPPED, RADIO_OVERRIDE};
    enum DIRECTION {DIR_HOLD, DIR_REV, DIR_FWD, DIR_MAX};

private:
    ROBOT_STATE robotState = ROBOT_IDLE;
    DIRECTION robotDirection = DIR_HOLD;

    float nearestObjectCM[DIR_MAX];
    
    EventTimer deterrenceTimer;
    uint8_t deterrenceCount = 0;

public:
    void handleMaxBotixReading(float distance, DIRECTION direction);  // needs to know what sensor
    void setTargetSpeed(float speed);
    void handleEncoderUpdate(const float movementCM);

    void SetDirection(DIRECTION dir);
    void SwitchDirections(void);

    void Arm(void);
    void Disarm(void);
    void Override(void);
    void FullStop(void);
};

extern Robot robot;