#pragma once

#include <ros.h>
#include <std_msgs/Int16.h>

/**
 * Utility functions for dealing with rangefinders.
*/

void setup_rangefinder(ros::NodeHandle& nh);
void processRangefinders(void);
