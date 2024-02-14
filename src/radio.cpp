#include "radio-ROS.h"

#include <Arduino.h>


void RadioPulse::Init(void (*isr)(void))
{
    Serial.println("RadioPulse::init()");
    
    pinMode(rcPin, INPUT);

    if(digitalPinToInterrupt(rcPin) != NOT_AN_INTERRUPT)
    {
        Serial.print(F("Attaching ISR to pin: "));
        Serial.println(rcPin);
        attachInterrupt(digitalPinToInterrupt(rcPin), isr, CHANGE);
    }

#ifdef __AVR_
    else if(digitalPinToPCInterrupt(echoPin) != NOT_AN_INTERRUPT)
    {
        Serial.print(F("Attaching PC_ISR to PCINT"));
        Serial.println(digitalPinToPCInterrupt(echoPin));
        attachPCInt(digitalPinToPCInterrupt(echoPin), isr);
    }
#endif

    else
    {
        Serial.println(F("Not an interrupt pin!"));
    }

    Serial.println("/RadioPulse::init()");
}

bool RadioPulse::GetPulseWidth(uint32_t& pulseWidth)
{
    bool newReading = false;
    
    if(state & PULSE_RECD)
    {
        noInterrupts();
        state &= ~PULSE_RECD;

        pulseWidth = pulseEnd - pulseStart;
        interrupts();

        newReading = true;
    }

    return newReading;
}
