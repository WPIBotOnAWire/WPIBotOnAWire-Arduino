#include "camera-ROS.h"

#include "robot.h"
#include <std_msgs/Int16.h>

// callback function for receiving status commands
void cbCameraDetection(const std_msgs::Int16& msg) 
{
    int16_t distance = msg.data;
    robot.handleCameraReading(distance, Robot::DIR_FWD);
}

ros::Subscriber<std_msgs::Int16> camera_sub("/bird_distance_camera", cbCameraDetection);

void init_camera_node(ros::NodeHandle& nh)
{
    nh.subscribe(camera_sub);
}
