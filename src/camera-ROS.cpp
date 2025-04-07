#include "camera-ROS.h"
#include "led-ROS.h"
#include "robot.h"
#include <std_msgs/Int16.h>

const float CAMERA_THRESHOLD = 100; // cm

// callback function for receiving status commands
void cbCameraDetection(const std_msgs::Int16& msg) 
{
    int16_t distance = msg.data;
    robot.handleCameraReading(distance, Robot::DIR_FWD);
    if(distance < CAMERA_THRESHOLD) setLED();
}

ros::Subscriber<std_msgs::Int16> camera_sub("/bird_distance_camera", cbCameraDetection);

void init_camera_node(ros::NodeHandle& nh)
{
    nh.subscribe(camera_sub);
}
