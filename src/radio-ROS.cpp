#include "radio-ROS.h"
#include "esc-samd21.h"
#include <std_msgs/Bool.h>

#define RADIO_OVERRIDE_PIN  12      // slot 5 of the radio receiver
#define RADIO_SPEED_PIN     13      // slot 1 of the radio receiver

std_msgs::Bool radioOverrideMsg;
ros::Publisher pubRadioOverride("/radio/override", &radioOverrideMsg);

RadioPulse radioOverride(RADIO_OVERRIDE_PIN);
void ISR_RADIO_OVERRIDE(void) {radioOverride.radioISR();}

RadioPulse radioSpeed(RADIO_SPEED_PIN);
void ISR_RADIO_SPEED(void) {radioSpeed.radioISR();}

void setupRadio(ros::NodeHandle& nh)
{
    DEBUG_SERIAL.println("setupRadio");
    nh.advertise(pubRadioOverride);

    radioOverride.Init(ISR_RADIO_OVERRIDE);
    radioSpeed.Init(ISR_RADIO_SPEED);
}

bool override = false;

void processRadio(void)
{
    uint32_t overridePulseLength = 0;
    if(radioOverride.GetPulseWidth(overridePulseLength))
    {
        if(overridePulseLength > 2000) 
        {
            DEBUG_SERIAL.print("Spike: ");
            DEBUG_SERIAL.println(overridePulseLength);
        }

        override = (overridePulseLength > 1900 && overridePulseLength < 2000);
        radioOverrideMsg.data = override;
        pubRadioOverride.publish(&radioOverrideMsg);
        if(override)
        {
            DEBUG_SERIAL.print("Override: ");
            DEBUG_SERIAL.print('\t');
            DEBUG_SERIAL.print(overridePulseLength);
            DEBUG_SERIAL.print('\n');
        }
    }

    uint32_t radioSpeedPulse = 0;
    if(radioSpeed.GetPulseWidth(radioSpeedPulse))
    {
        if(override)
        {
            if(radioSpeedPulse <= 1490 || radioSpeedPulse >= 1510)
                escPair.SetOverridePulse(radioSpeedPulse);
            else escPair.SetOverridePulse(1500);
        }
    }
}