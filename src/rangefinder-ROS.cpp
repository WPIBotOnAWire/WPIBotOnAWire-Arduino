#include "rangefinder-ROS.h"
#include <MaxBotix.h>
#include <std_msgs/UInt16.h>

#include "robot.h"

std_msgs::UInt16 mbFrontCM; //in cm
ros::Publisher pubMBfront("/rangefinder/fore/MB", &mbFrontCM);

MaxBotixPulse mbFront(A3);
void ISR_MB_FRONT(void) {mbFront.mbISR();}

std_msgs::UInt16 mbAftCM; //in cm
ros::Publisher pubMBaft("/rangefinder/aft/MB", &mbAftCM);

MaxBotixPulse mbAft(A0);
void ISR_MB_AFT(void) {mbAft.mbISR();}

void setup_rangefinders(ros::NodeHandle& nh)
{
  nh.advertise(pubMBfront);
  nh.advertise(pubMBaft);

  mbFront.init(ISR_MB_FRONT);
  mbAft.init(ISR_MB_AFT);
}


void processRangefinders(void)
{
  uint16_t frontDist = 0;
  if(mbFront.getDistance(frontDist))
  {
    mbFrontCM.data = frontDist / 10; //raw reading is in mm
    pubMBfront.publish(&mbFrontCM);
    robot.handleMaxBotixReading(frontDist / 10, Robot::DIR_FWD);
  }

  uint16_t aftDist = 0;
  if(mbAft.getDistance(aftDist))
  {
    mbAftCM.data = aftDist / 10; //raw reading is in mm
    pubMBaft.publish(&mbAftCM);
    robot.handleMaxBotixReading(aftDist / 10, Robot::DIR_REV);
  }
}

