#include "rangefinder.h"

std_msgs::Int16 mbFrontMM; //in mm; use negative for errors?
ros::Publisher pubMBfront("/rangefinder/front/MB", &mbFrontMM);

void setup_rangefinder(ros::NodeHandle& nh)
{
  //nh.getHardware()->setBaud(9600); //struggling to understand this one...
  nh.advertise(pubMBfront);
  
  // // wait controller to be connected
  // while (!nh.connected()){
  //   nh.spinOnce();
  // }
  // // if initialization failed - write message and freeze
  // if (!sensor.begin()) {
  //   nh.logwarn("Failed to setup VL53L0X sensor");
  //   while(1);
  // }
  // nh.loginfo("VL53L0X API serial node started");
  // // fill static range message fields
  
}

