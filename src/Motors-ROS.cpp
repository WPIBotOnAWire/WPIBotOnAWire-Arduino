#include "Motors-ROS.h"

#include "encoder.h"

#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include "esc-samd21.h"

#include "robot.h"

#define Serial SerialUSB

#define Kp 1
#define Ki 0.01

// ENCODER AND PID CONSTANTS 
#define ENCODER_PIN1           8
#define ENCODER_PIN2           9

//#define WHEEL_RADIUS            0.0210  // meters // 0.825/39.37; //inches to m
//#define WHEEL_CIRCUMFRANCE      0.132   // meters // 2.0*PI*WHEEL_RADIUS;
//#define PPR                     1024.0  // pulses per revolution of the encoder
#define METERS_PER_TICK         0.0001286 // meters/tick
//#define CM_PER_TICK             0.01286 // cm/tick // maybe use this?
#define LOOP_RATE_MS            50

QuadEncoder<ENCODER_PIN1,ENCODER_PIN2> encoder; 

std_msgs::Int32 enc_val;
ros::Publisher pub_enc("/encoder/count", &enc_val);

std_msgs::Float32 speed_enc;
ros::Publisher pub_speed("/encoder/meters_per_second", &speed_enc);

ESCDirect escPair;

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

void init_motors(ros::NodeHandle& nh)
{
    //sets up the ESCs and battery info to allow them to spin
    escPair.Init();

    //escPair.Calibrate(); 
    escPair.Arm();

    //escPair.Stop();

    nh.subscribe(motor_sub);
}

//  encoder stuff
void setup_encoder(ros::NodeHandle& nh)
{
  encoder.Init();
  nh.advertise(pub_enc);
  nh.advertise(pub_speed);
}

/**
 * Reads the current encoder count (count is updated in an ISR) and publishes the result.
 * Nominally tested and working with a quadrature encoder.
*/
void processEncoders(void)
{
  static uint32_t lastEncoderReport = 0;
  uint32_t currTime = millis();

  if(currTime - lastEncoderReport > LOOP_RATE_MS) 
  {
    lastEncoderReport = currTime;

    int32_t currTicks = encoder.TakeSnapshot();
    int32_t delta = encoder.CalcDelta();

    // DEBUG_SERIAL.println(currTicks);

    enc_val.data = currTicks;
    pub_enc.publish(&enc_val);

    float movementMeters = delta * METERS_PER_TICK;
    robot.handleEncoderUpdate(movementMeters * 100.0);

    float speedMetersPerSecond = movementMeters / (float) LOOP_RATE_MS;
    speed_enc.data = speedMetersPerSecond;
    pub_speed.publish(&speed_enc);

    // int16_t error = targetSpeedTicksPerInterval - delta;
    // static int16_t sumError = 0;
    // sumError += error;

  }  
}