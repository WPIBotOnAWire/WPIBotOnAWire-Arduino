#include "rangefinder.h"

void rangefinder::init()
{
    rh.init();
    rh.rfSetup();
    // wait controller to be connected
    while (!rh.connected())
    {
        rh.spin();
    }
    // if initialization failed - write message and freeze
    if (!sensor.begin())
    {
        rh.logwarn("Failed to setup VL53L0X sensor");
        while (1)
            ;
    }
    rh.loginfo("VL53L0X API serial node started");
    // fill static range message fields
}

void rangefinder::range()
{
    if ((millis() - range_timer) > 50)
    {
        sensor.rangingTest(&measure, false);
        if (measure.RangeStatus != 4)
        {                                                                // phase failures have incorrect data
            rh.publishFrontRF((float)measure.RangeMilliMeter / 1000.0f); // convert mm to m
            float p = measure.RangeMilliMeter;
            char result[20];
            dtostrf(p, 20, 5, result);
            rh.logwarn(result);
        }
        else
        {
            rh.publishFrontRF((float)-999999999);
            rh.logwarn("Out of range"); // if out of range, don't send message
        }
        range_timer = millis();
    }
}
