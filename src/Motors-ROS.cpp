#include "Motors-ROS.h"

//#include "encoder.h"
#include "ESmotor.h"

#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include "robot.h"

#define Serial SerialUSB

std_msgs::Int32 enc_val;
ros::Publisher pub_enc("/encoder/count", &enc_val);

std_msgs::Float32 speed_enc;
ros::Publisher pub_speed("/encoder/meters_per_second", &speed_enc);

ESMotor esMotor(8, 9);

int16_t targetSpeedTicksPerInterval = 0;
int16_t currentTargetTicksPerInterval = 0;

// callback function for receiving speed commands
void cbTargetSpeed(const std_msgs::Float32& msg) 
{
  float targetSpeed = msg.data;
  
//  targetSpeedTicksPerInterval = targetSpeed * (LOOP_RATE_MS / 1000.0) / METERS_PER_TICK;
//    escPair.SetSpeed(msg.data); //this is percent full speed; should be m/s?
      // float effort = Kp * error + Ki * sumError;
    
    int16_t effort = constrain(targetSpeed, -100, 100);
    //escPair.SetTargetSpeed(effort); // moving onboard; ignore ROS commands
}

ros::Subscriber<std_msgs::Float32> motor_sub("/target_speed_meters_per_sec", cbTargetSpeed);

/**
 * Initialize the motors.
 */
void init_motors(ros::NodeHandle& nh)
{
    esMotor.Init();
    esMotor.Arm();

    nh.subscribe(motor_sub);
}

/**
 * Advertise encoder, speed.
 */
void setup_encoder(ros::NodeHandle& nh)
{
  nh.advertise(pub_enc);
  nh.advertise(pub_speed);
}

void updateMotors(void) {esMotor.UpdateMotors();}