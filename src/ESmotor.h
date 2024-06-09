#pragma once

#include <Arduino.h>

/**
 * New BLDC motor with built-in power, encoder, etc
 * 
 * https://www.robotshop.com/products/36d-long-life160brushless-dc-geared-motor-24v-1300rpm
 * 
 */
class ESMotor
{
public:
    enum MOTOR_STATE {IDLE, ARMED, OVERRIDE};

    volatile int8_t readyToPID = 0;

private:
    uint8_t directionPin = -1;
    MOTOR_STATE motorState = IDLE;

    float targetSpeed = 0;
    float currentSetPoint = 0;

    void SetEffort(int16_t match);

public:
    void Init(void);
    MOTOR_STATE Arm(void);
    bool isArmed(void) {return motorState == ARMED;}
    MOTOR_STATE UpdateMotors(void);

    MOTOR_STATE SetTargetSpeed(int16_t pct);
    MOTOR_STATE SetTargetSpeedMetersPerSecond(float speedMPS);
    MOTOR_STATE SetOverridePulse(uint32_t pulseWidth)
    {
        //WriteMicroseconds(pulseWidth);
        return motorState = OVERRIDE;
    }

    void EStop(void) 
    {
        targetSpeed = 0; 
        currentSetPoint = 0; 
        //WriteMicroseconds(oStop); 
        motorState = IDLE;
    }

};

extern ESMotor esMotor;