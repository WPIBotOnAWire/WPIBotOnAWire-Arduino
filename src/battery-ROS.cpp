#include "battery-ROS.h"

#include <Adafruit_INA260.h>
#include <sensor_msgs/BatteryState.h>

// battery Monitor
Adafruit_INA260 bat_monitor = Adafruit_INA260();

sensor_msgs::BatteryState bat_msg;
ros::Publisher pub_bat_level("/battery", &bat_msg);

void initBatteryMonitor(ros::NodeHandle& nh)
{
    bat_monitor.begin();

    nh.advertise(pub_bat_level);
}

void processBatteryMonitor(void)
{
    static uint32_t lastBatteryReport = 0;
    uint32_t currTime = millis();

    if(currTime - lastBatteryReport > 500) 
    {
        lastBatteryReport = currTime;

        //THESE GET THE BATTERY INFO AND ARE NEEDED TO MAKE MOTOR SPIN ????
        bat_msg.voltage = bat_monitor.readBusVoltage();
        bat_msg.current = bat_monitor.readCurrent();

        pub_bat_level.publish(&bat_msg);
    }
}
