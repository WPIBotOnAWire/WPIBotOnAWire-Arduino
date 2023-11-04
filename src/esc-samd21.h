#pragma once

#include <Arduino.h>
#include <event_timer.h>

class ESCDirect
{
private:
    enum MOTOR_STATE {IDLE, AMRING, CAL_LOW, CAL_HIGH, STOPPED, ACTIVE};
    MOTOR_STATE motorState = IDLE;

    // Calibration
    int oMin = 1000; 
    int oMid = 1500;
    int oStop = 1500;
    int oMax = 2000;
    int oArm = 500;

    uint32_t calibrationDelay = 3000;	// Calibration delay (millisecond)

    void WriteMicroseconds(uint16_t uSec);
    void WriteMin(uint32_t duration) {}

public:
    void Init(void);
    void Arm(void);
    void Stop(void) {WriteMicroseconds(oStop);}
    void SetSpeed(int16_t pct);
    void Calibrate(void);
};
