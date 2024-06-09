#include "radio-ROS.h"
#include "ESmotor.h"
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

int8_t directionCount = 0;

void processRadio(void)
{
    uint32_t overridePulseLength = 0;
    if(radioOverride.GetPulseWidth(overridePulseLength))
    {
        bool prev_override = override; // I don't like this here -- should go at end?

        if(overridePulseLength >= 2010) 
        {
            DEBUG_SERIAL.print("Spike: ");
            DEBUG_SERIAL.println(overridePulseLength);
        }

        /**
         * This bit of code makes it so we have to get N overrides in a row to switch over.
         * Been having issues with spikes.
        */
        if(overridePulseLength > 1900 && overridePulseLength < 2010) 
        {
            if(++overrideCount >= minOverrideCount)
            {
                overrideCount = minOverrideCount; // cap it so we don't roll over
                override = true;
            }
        }
        else if (overridePulseLength < 1200 && overridePulseLength > 900)
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
        if(override && !prev_override) esMotor.Arm();
    }

    uint32_t radioSpeedPulse = 0;
    if(radioSpeed.GetPulseWidth(radioSpeedPulse))
    {
        /**
         * If we're in override mode, use the speed pulse to directly command the motors.
        */
        if(override)
        {
            if(radioSpeedPulse > 2000 || radioSpeedPulse < 1000) {} //ignore the spikes
            esMotor.SetOverridePulse(radioSpeedPulse);
        }

        /**
         * If we're not in override mode, use the pulses to command the patrol direction.
        */
        else
        {
            if(radioSpeedPulse < 1200) directionCount--;
            else if(radioSpeedPulse > 1800) directionCount++;
            else directionCount = 0; //but don't reset direction; just avoid spikes

            if(directionCount < -5)
            {
                directionCount = -5;
                robot.SetDirection(Robot::DIR_REV);
            }

            else if(directionCount > 5)
            {
                directionCount = 5;
                robot.SetDirection(Robot::DIR_FWD);
            }
        }
    }
}