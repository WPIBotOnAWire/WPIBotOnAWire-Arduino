#include "status-ROS.h"

#include <std_msgs/String.h>
#include "esc-samd21.h"

std_msgs::String status;
//ros::Publisher pub_bat_level("/battery", &bat_msg);

// callback function for receiving status commands
void cbStatus(const std_msgs::String& msg) 
{
    String message = msg.data;
    // SerialUSB.println(message);
    if(message == String("Arm"))
    {
        escPair.Arm();
    }

    else if(message == String("Emergency Stop"))
    {
        escPair.EStop();
    }
}

ros::Subscriber<std_msgs::String> status_sub("/status", cbStatus);

void init_status(ros::NodeHandle& nh)
{
    nh.subscribe(status_sub);
}
