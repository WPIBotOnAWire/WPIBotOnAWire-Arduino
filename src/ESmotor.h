#pragma once

#include <Arduino.h>

/**
 * New BLDC motor with built-in driver, encoder, etc
 * 
 * https://www.robotshop.com/products/36d-long-life160brushless-dc-geared-motor-24v-1300rpm
 * 
 */
class ESMotor
{
private:
    uint8_t directionPin = -1;
    uint8_t encoderPin = -1;

    enum MOTOR_MODE {MOTOR_DIRECT, MOTOR_AUTO};
    MOTOR_MODE motorMode = MOTOR_DIRECT;

    bool isArmed = false;

    volatile int16_t encoderCount = 0;
    volatile int16_t snapshotCount = 0;
    volatile int16_t previousCount = 0;

    volatile int8_t readyToPID = 0;
    uint32_t lastMotorUpdate = 0;

    const float Kp = 2;
    const float Ki = 0.2;

    float sumError = 0;
    //float integralCap = 400 / Ki; // we'll prevent build up in the PI routine

    /**
     * targetSpeed is the speed we want to be going, in units of encoder [ticks/20 ms interval].
     * 
     * currentSetPoint is the currently commanded speed. The currentSetPoint varies more slowly
     * to avoid jerk.
     * 
     * The maximum speed is ~20 ticks/interval, so it takes about a half second to fully ramp up,
     * which is reasonable.
     */
    float targetSpeed = 0;
    float currentSetPoint = 0;
    volatile int8_t direction = 0;

    void SetEffort(int16_t match);

    /**
     * Technically just open-loop, but might add info from IMU later.
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

public:
    ESMotor(int8_t dirPin = -1, int8_t encPin = -1) 
        {directionPin = dirPin; encoderPin = encPin;}

    void Init(void);
    bool Arm(void);
    bool Disarm(void);
    void SetAuto(void) { motorMode = MOTOR_AUTO; targetSpeed = currentSetPoint = 0; sumError = 0; }
    void EStop(void) 
    {
        Disarm();
    }

    void FullStop(void) 
    {
        targetSpeed = 0; 
        currentSetPoint = 0; 
        sumError = 0;
    }

    void UpdateMotors(void);

    bool SetTargetSpeedMetersPerSecond(float speedMPS);

    MOTOR_MODE SetPulseDirect(uint32_t pulseWidth)
    {
        // For the E-S Motor, we must convert RC pulses to speed commands
        // 1000 -> -400; 2000 -> 400
        // map(long x, long in_min, long in_max, long out_min, long out_max)
        int16_t speedCommand = map(pulseWidth, 1100, 1900, -400, 400);
        SetEffort(speedCommand);
        return motorMode = MOTOR_DIRECT;
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