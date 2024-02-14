#include "radio-ROS.h"
#include <std_msgs/Bool.h>

#define RADIO_OVERRIDE_PIN  13
#define RADIO_SPEED_PIN  12

std_msgs::Bool radioOverrideMsg;
ros::Publisher pubRadioOverride("/radio/override", &radioOverrideMsg);

RadioPulse radioOverride(RADIO_OVERRIDE_PIN);
void ISR_RADIO_OVERRIDE(void) {radioOverride.radioISR();}

RadioPulse radioSpeed(RADIO_SPEED_PIN);
void ISR_RADIO_SPEED(void) {radioSpeed.radioISR();}

void setupRadio(ros::NodeHandle& nh)
{
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
        override = (overridePulseLength > 1500);
        radioOverrideMsg.data = override;
        pubRadioOverride.publish(&radioOverrideMsg);
    }

    if(override)
    {
        uint32_t radioSpeedPulse = 0;
        if(radioSpeed.GetPulseWidth(radioSpeedPulse))
        {
            
        }
    }
}