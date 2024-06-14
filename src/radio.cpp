#include "radio-ROS.h"

#include <Arduino.h>

void RadioPulse::Init(void (*isr)(void))
{
    pinMode(rcPin, INPUT_PULLUP);

    if(digitalPinToInterrupt(rcPin) != NOT_AN_INTERRUPT)
    {
        DEBUG_SERIAL.print(F("Attaching ISR to pin: "));
        DEBUG_SERIAL.println(rcPin);
        attachInterrupt(digitalPinToInterrupt(rcPin), isr, CHANGE);
    }

#ifdef __AVR_
    else if(digitalPinToPCInterrupt(echoPin) != NOT_AN_INTERRUPT)
    {
        DEBUG_SERIAL.print(F("Attaching PC_ISR to PCINT"));
        DEBUG_SERIAL.println(digitalPinToPCInterrupt(echoPin));
        attachPCInt(digitalPinToPCInterrupt(echoPin), isr);
    }
#endif

    else
    {
        DEBUG_SERIAL.println(F("Not an interrupt pin!"));
    }

    DEBUG_SERIAL.println("/RadioPulse::init()");
}

bool RadioPulse::GetPulseWidth(uint32_t& pulseWidth)
{
    bool newReading = false;
    
    if(state & PULSE_RECD)
    {
        noInterrupts();
        state &= ~PULSE_RECD;

        // Check for rollover
        // Should really add MOTOR_UPDATE_MS * 1000U
        if(pulseStart > pulseEnd) pulseEnd += 20000; 
        
        pulseWidth = pulseEnd - pulseStart;
        interrupts();

        newReading = true;
    }

    return newReading;
}
