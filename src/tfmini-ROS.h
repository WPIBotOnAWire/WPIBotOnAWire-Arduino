#pragma once

#include <ros.h>
#include <Arduino.h>
#include <Rangefinder.h>

class TFmini : public Rangefinder
{
protected:
    uint8_t checkSum = 0;
    uint8_t tfMiniArray[9]; //array for receiving data from the UART
    uint8_t tfIndex = 0; //for counting bytes

    bool handleUART(uint8_t b);
    HardwareSerial* serial;

public:
    void setupTFmini(void);
    bool getDistance(uint16_t& distance);
};

void setupTFminis(ros::NodeHandle& nh);
void processTFminis(void);