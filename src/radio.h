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
        // Don't use micros(), as it glitches (what the hell?)
        // TCC1 is set up for 1us per tick (with period the control interval)

        // Step 1: Issue READYSYNC command
        TCC1->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;

        // Step 2: Wait until the command is fully executed
        while (TCC1->SYNCBUSY.bit.CTRLB); // or while (TCC1->SYNCBUSY.reg);

        // Step 3: Now we can read the value of the COUNT register
        uint32_t count = TCC1->COUNT.reg;

        if(digitalRead(rcPin))    //transitioned to HIGH
        {
            pulseStart = count;
            state &= ~PULSE_RECD; //cancel pulse; could raise a warning flag
        }

        else                    //transitioned to LOW
        {
            pulseEnd = count;
            state |= PULSE_RECD;
        } 
    }
};
