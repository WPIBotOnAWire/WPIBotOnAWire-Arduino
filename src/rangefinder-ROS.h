#pragma once

#include <ros.h>

/**
 * Utility functions for dealing with rangefinders.
*/

void setup_rangefinders(ros::NodeHandle& nh);
void processRangefinders(void);
