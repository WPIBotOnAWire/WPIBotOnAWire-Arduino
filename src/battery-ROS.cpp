#include "battery-ROS.h"

#include <Adafruit_INA260.h>

#include <ros.h>
#include <sensor_msgs/BatteryState.h>

// battery Monitor
Adafruit_INA260 bat_monitor = Adafruit_INA260();

sensor_msgs::BatteryState bat_msg;
ros::Publisher pub_bat_level("/battery", &bat_msg);

void updateBat(void)
{
    //THESE GET THE BATTERY INFO AND ARE NEEDED TO MAKE MOTOR SPIN ^^^^
    bat_msg.voltage = bat_monitor.readBusVoltage();
    bat_msg.current = bat_monitor.readCurrent();
    pub_bat_level.publish(&bat_msg);
}