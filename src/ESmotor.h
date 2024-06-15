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

    const float Kp = 2;
    const float Ki = 0.2;

    float sumError = 0;
    float integralCap = 400 / Ki; // doesn't need to be this big with the feedforward term?

    /**
     * Really just open-loop, but might add info from IMU later.
     * Target is in ticks/interval.
     */
    float FeedForward(float target)
    {
        float ff = 0;

        if(target > 0)
            ff = 160 + 10 * target;
        else if (target < 0)
            ff = -160 + 10 * target;

        return ff;
    }

private:
    uint8_t directionPin = -1;
    uint8_t encoderPin = -1;
    MOTOR_STATE motorState = IDLE;

    /**
     * targetSpeed is the speed we want to be going, in units of encoder [ticks/20 ms interval].
     * 
     * currentSetPoint is the currently commanded speed. The currentSetPoint varies more slowly
     * to avoid jerk.
     * 
     * The maximum speed is ~25 ticks/interval, so it takes about a half second to fully ramp up,
     * which is reasonable.
     */
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
        int16_t speedCommand = map(pulseWidth, 1100, 1900, -400, 400);
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