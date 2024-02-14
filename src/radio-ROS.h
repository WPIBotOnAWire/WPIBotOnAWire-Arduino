#pragma once

#include <ros.h>
#include <Arduino.h>
#include <radio.h>

void setupRadio(ros::NodeHandle& nh);
void processRadio(void);