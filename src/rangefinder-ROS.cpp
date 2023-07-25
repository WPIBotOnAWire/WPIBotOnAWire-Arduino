#include "rangefinder-ROS.h"
#include <MaxBotix.h>
#include <std_msgs/UInt16.h>

std_msgs::UInt16 mbFrontCM; //in cm
ros::Publisher pubMBfront("/rangefinder/front/MB", &mbFrontCM);

MaxBotixPulse mbFront(8);
void ISR_MB_FRONT(void) {mbFront.mbISR();}

void setup_rangefinder(ros::NodeHandle& nh)
{
  //nh.getHardware()->setBaud(9600); //struggling to understand this one...
  nh.advertise(pubMBfront);

  mbFront.init(ISR_MB_FRONT);
  
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


void processRangefinders(void)
{
  float frontDist = 0;
  if(mbFront.getDistance(frontDist))
  {
    mbFrontCM.data = frontDist;
    pubMBfront.publish(&mbFrontCM);
  }

  //   if ((millis()-range_timer) > 50){
  //   sensor.rangingTest(&measure, false);
  //   if (measure.RangeStatus != 4) {  // phase failures have incorrect data
  //       rf_front_val.data = (float) measure.RangeMilliMeter/1000.0f; // convert mm to m
  //       pub_rf_front.publish(&rf_front_val);
  //       float p = measure.RangeMilliMeter /1000.0f; // m
  //       //char result[20];
  //       //dtostrf(p, 20, 5, result);
  //       // nh.logwarn(result);
  //   } else {
  //     rf_front_val.data = (float) -999999999;
  //     pub_rf_front.publish(&rf_front_val);
  //     // nh.logwarn("Out of range"); // if out of range, don't send message
  //   }
  //   range_timer =  millis();    
  // }
}

