#include "status-ROS.h"

#include <std_msgs/String.h>
#include "ESMotor.h"

std_msgs::String status;
//ros::Publisher pub_bat_level("/battery", &bat_msg);

// callback function for receiving status commands
void cbStatus(const std_msgs::String& msg) 
{
    String message = msg.data;
    if(message == String("Arm"))
    {
        esMotor.Arm();
    }

    else if(message == String("Emergency Stop"))
    {
        esMotor.EStop();
    }
}

ros::Subscriber<std_msgs::String> status_sub("/status", cbStatus);

void init_status(ros::NodeHandle& nh)
{
    nh.subscribe(status_sub);
}
