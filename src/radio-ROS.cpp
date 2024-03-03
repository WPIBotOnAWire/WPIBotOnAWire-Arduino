#include "radio-ROS.h"
#include "esc-samd21.h"
#include <std_msgs/Bool.h>

#include "robot.h"

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

const uint8_t minOverrideCount = 5; // to avoid spikes, which are annoying
int8_t overrideCount = 0;
bool override = false;

void processRadio(void)
{
    uint32_t overridePulseLength = 0;
    if(radioOverride.GetPulseWidth(overridePulseLength))
    {
        bool prev_override = override;

        if(overridePulseLength > 2000) 
        {
            DEBUG_SERIAL.print("Spike: ");
            DEBUG_SERIAL.println(overridePulseLength);
        }

        /**
         * This bit of code makes it so we have to get N overrides in a row to switch over.
         * Been having issues with spikes.
        */
        if(overridePulseLength > 1900 && overridePulseLength < 2000) 
        {
            if(++overrideCount >= minOverrideCount)
            {
                overrideCount = minOverrideCount; // cap it so we don't roll over
                override = true;
            }
        }
        else if (overridePulseLength < 1200)
        {
            if(--overrideCount <= 0)
            {
                override = false;
                overrideCount = 0;
            }
        }

        radioOverrideMsg.data = override;
        pubRadioOverride.publish(&radioOverrideMsg);

        // if(override && !prev_override)
        // {
        //     DEBUG_SERIAL.print("Override: ");
        //     DEBUG_SERIAL.print('\t');
        //     DEBUG_SERIAL.print(overridePulseLength);
        //     DEBUG_SERIAL.print('\n');
        // }

        if(prev_override && !override) robot.Arm();
    }

    uint32_t radioSpeedPulse = 0;
    if(radioSpeed.GetPulseWidth(radioSpeedPulse))
    {
        if(override)
        {
            if(radioSpeedPulse > 2000 || radioSpeedPulse < 1000) {} //ignore the spike
            else if(radioSpeedPulse <= 1490 || radioSpeedPulse >= 1510)
                escPair.SetOverridePulse(radioSpeedPulse);
            else escPair.SetOverridePulse(1500);
        }
    }
}