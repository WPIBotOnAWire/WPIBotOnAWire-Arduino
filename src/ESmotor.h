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

    volatile int16_t encoderCount = 0;

private:
    volatile int16_t snapshotCount = 0;
    volatile int16_t previousCount = 0;
    volatile int8_t readyToPID = 0;

    int16_t sumError = 0;
    int16_t integralCap = 10000; // need to size...

    float Kp = 1;
    float Ki = 0.1;

private:
    uint8_t directionPin = -1;
    uint8_t encoderPin = -1;
    MOTOR_STATE motorState = IDLE;

    float targetSpeed = 0;
    float currentSetPoint = 0;
    volatile int8_t direction = 0;

    void SetEffort(int16_t match);

public:
    ESMotor(int8_t dirPin = -1, int8_t encPin = -1) 
        {directionPin = dirPin; encoderPin = encPin;}

    void Init(void);
    MOTOR_STATE Arm(void);
    MOTOR_STATE Disarm(void);

    // bool isArmed(void) {return motorState == ARMED;}
    MOTOR_STATE UpdateMotors(void);

    MOTOR_STATE SetTargetSpeed(int16_t pct);
    MOTOR_STATE SetTargetSpeedMetersPerSecond(float speedMPS);
    MOTOR_STATE SetOverridePulse(uint32_t pulseWidth)
    {
        // For the E-S Motor, we must convert RC pulses to speed commands
        // 1000 -> -400; 2000 -> 400
        // map(long x, long in_min, long in_max, long out_min, long out_max)
        int16_t speedCommand = map(pulseWidth, 1000, 2000, -400, 400);
        SetEffort(speedCommand);
        return motorState = OVERRIDE;
    }

    void EStop(void) 
    {
        targetSpeed = 0; 
        currentSetPoint = 0; 
        //WriteMicroseconds(oStop); 
        motorState = IDLE;
    }

    void handleEncoderISR(void) {encoderCount += direction;}
    void TakeSnapshot(void)
    {
        snapshotCount = encoderCount;
        readyToPID++;
    }

    int16_t CalcEncoderSpeed(void)
    {
        noInterrupts();
        int16_t count = snapshotCount;
        interrupts();

        int16_t speed = count - previousCount;
        previousCount = count;

#ifdef __ENC_DEBUG__
        DEBUG_SERIAL.print(count);
        DEBUG_SERIAL.print('\t');
        DEBUG_SERIAL.print(previousCount);
        DEBUG_SERIAL.print('\n');
#endif
        return speed;
    }
};

extern ESMotor esMotor;