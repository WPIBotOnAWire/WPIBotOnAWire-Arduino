#pragma once

#include <ros.h>

/**
 * Utility functions for dealing with rangefinders.
*/

void setup_rangefinder(ros::NodeHandle& nh);
void processRangefinders(void);
