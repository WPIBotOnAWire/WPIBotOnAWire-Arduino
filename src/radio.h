#include <Arduino.h>

#define PULSE_RECD 0x01
//#define PULSE_HIGH 0x02

class RadioPulse
{
private:
    uint8_t rcPin;
    volatile uint8_t state  = 0;

    volatile uint32_t pulseStart = 0;
    volatile uint32_t pulseEnd = 0;

public:
    RadioPulse(uint8_t pin) {rcPin = pin;}
    virtual void Init(void (*isr)(void));

    virtual bool GetPulseWidth(uint32_t& pulseWidth);

    inline void radioISR(void) volatile
    {
        if(digitalRead(rcPin))    //transitioned to HIGH
        {
            pulseStart = micros();
            state &= ~PULSE_RECD; //cancel pulse; could raise a warning flag
        }

        else                    //transitioned to LOW
        {
            pulseEnd = micros();
            state |= PULSE_RECD;
        } 
    }
};
