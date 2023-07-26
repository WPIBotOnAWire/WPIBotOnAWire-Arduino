#pragma once

#include <Arduino.h>

class ESCDirect
{
private:
    // Calibration
    int oMin = 1000; 
    int oMid = 1500;
    int oStop = 1500;
    int oMax = 2000;
    int oArm = 500;

    uint32_t calibrationDelay = 8000;	// Calibration delay (millisecond)

    void WriteMicroseconds(uint16_t uSec);

public:
    void Init(void);
    void Arm(void);
    void Stop(void) {WriteMicroseconds(oStop);}
    void SetSpeed(int16_t pct);
};
