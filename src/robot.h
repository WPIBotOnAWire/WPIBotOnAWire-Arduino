#pragma once

#include <Arduino.h>
#include "event_timer.h"

class Robot
{
public:
    enum ROBOT_STATE {ROBOT_IDLE, ROBOT_PATROLLING, ROBOT_APPROACHING, ROBOT_DETERRING, 
                        ROBOT_STOPPED, RADIO_TELEOP, JETSON_COMMAND};
    enum DIRECTION {DIR_HOLD, DIR_REV, DIR_FWD};

private:
    ROBOT_STATE robotState = ROBOT_IDLE;
    DIRECTION robotDirection = DIR_HOLD;

    float nearestObjectCM[3]; // ignore hold
    
    EventTimer deterrenceTimer;
    uint8_t deterrenceCount = 0;

public:
    void handleMaxBotixReading(float distance, DIRECTION direction);  // needs to know what sensor
    void setTargetSpeed(float speed);
    void handleEncoderUpdate(const float movementCM);

    void SetDirection(DIRECTION dir);
    void SwitchDirections(void);

    void SetAuto(void);
    void SetTeleop(void);
    void Disarm(void);
    void FullStop(void);

    bool CheckDeterrenceTimer(void);
    void HandleDeterrenceTimer(void);

    void loop(void)
    {
        if(CheckDeterrenceTimer()) HandleDeterrenceTimer();
    }
};

extern Robot robot;