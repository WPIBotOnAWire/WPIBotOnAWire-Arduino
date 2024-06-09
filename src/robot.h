#pragma once

#include <Arduino.h>

class Robot
{
public:
    enum ROBOT_STATE {ROBOT_IDLE, ROBOT_PATROLLING, ROBOT_APPROACHING, ROBOT_DETERRING, RADIO_OVERRIDE};
    enum DIRECTION {DIR_HOLD, DIR_REV, DIR_FWD, DIR_MAX};

private:
    ROBOT_STATE robotState = ROBOT_IDLE;
    DIRECTION robotDirection = DIR_HOLD;

    float nearestObjectCM[DIR_MAX];

public:
    void handleMaxBotixReading(float distance, DIRECTION direction);  // needs to know what sensor
    void setTargetSpeed(float speed);
    void handleEncoderUpdate(const float movementCM);

    void SetDirection(DIRECTION dir);

    void Arm(void);
    void Disarm(void);
    void Override(void);
};

extern Robot robot;