#pragma once

#include <ros.h>

void init_motors(ros::NodeHandle& nh);
void setup_encoder(ros::NodeHandle& nh);

void processEncoders(void);


