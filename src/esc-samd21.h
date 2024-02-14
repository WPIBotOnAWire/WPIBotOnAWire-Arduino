#pragma once

#include <Arduino.h>
#include <event_timer.h>

// Motor Constants -- TODO: pass to constructor
// fullforward = 2000, fullback = 1000
#define MOTOR_FULL_REV    1000
#define MOTOR_STOP        1500
#define MOTOR_FULL_FWD    2000

/**
 * There is some confusion about how the ESCs are set up. The datasheet has
 * lots of options for calibrating, but it appears they have a fixed calibration range (1000 - 2000us).
 * 
 * Experiments show that you only have to set the signal to 1500us for a 
 * 'few moments' and it beep-a-leeps at you.
 * 
 * So, we'll remove the Calibration function and just default to 1500us
 * in the Init() function. Then we'll need a timestamp for checking if it's
 * armed and call it good.
 */

class ESCDirect
{
public:
    enum MOTOR_STATE {IDLE, ARMED, OVERRIDE};

private:
    float targetSpeed = 0;
    float currentSetPoint = 0;

    MOTOR_STATE motorState = IDLE;
    EventTimer motorTimer;

    // Calibration
    int oMin = MOTOR_FULL_REV; 
    int oMid = MOTOR_STOP;
    int oStop = MOTOR_STOP;
    int oMax = MOTOR_FULL_FWD;

    uint32_t armingDelay = 3000;	// Calibration delay (millisecond)

    void WriteMicroseconds(uint16_t uSec);

public:
    void Init(void);
    MOTOR_STATE Arm(void);
    bool isArmed(void) {return motorState == ARMED;}
    
    MOTOR_STATE SetTargetSpeed(int16_t pct);
    MOTOR_STATE SetOverridePulse(uint32_t pulseWidth)
    {
        WriteMicroseconds(pulseWidth);
        return motorState = OVERRIDE;
    }
//    void Stop(void) {currentSpeed = 0; WriteMicroseconds(oStop);}
    void EStop(void) 
    {
        targetSpeed = 0; 
        currentSetPoint = 0; 
        WriteMicroseconds(oStop); 
        motorState = IDLE;
    }

    ESCDirect::MOTOR_STATE UpdateMotors(void);
};

extern ESCDirect escPair;