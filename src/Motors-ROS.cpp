#include "Motors-ROS.h"

#include "encoder.h"

#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include "esc-samd21.h"

#define Serial SerialUSB

// Motor Constants -- TODO: pass to constructor
// fullforward = 2000, fullback = 1000
#define MOTOR_FULLFORWARD    2000
#define MOTOR_FULLBACK       1000
#define MOTOR_STOP           1500

// ENCODER AND PID CONSTANTS -- NEED TO ADJUST INTERRUPTS TO NOT INTERFERE WITH I2C!!!!
#define ENCODER_PIN1           8
#define ENCODER_PIN2           9

#define WHEEL_RADIUS            0.0210  // meters // 0.825/39.37; //inches to m
#define WHEEL_CIRCUMFRANCE      0.0132  // meters // 2.0*PI*WHEEL_RADIUS;
#define PPR                     1024.0  // pulses per revolution of the encoder
#define METERS_PER_TICK         0.0001286 // meters/tick
#define CM_PER_TICK             0.01286 // cm/tick // maybe use this?

QuadEncoder<ENCODER_PIN1,ENCODER_PIN2> encoder; 

std_msgs::Int32 enc_val;
ros::Publisher pub_enc("/encoder/count", &enc_val);

std_msgs::Float32 speed_enc;
ros::Publisher pub_speed("/encoder/meters_per_interval", &speed_enc);

ESCDirect escPair;

int16_t targetSpeed = 0;

// callback function for receiving speed commands
void cbTargetSpeed(const std_msgs::Int16& msg) 
{
  targetSpeed = msg.data;
//    escPair.SetSpeed(msg.data); //this is percent full speed; should be m/s?
}

ros::Subscriber<std_msgs::Int16> motor_sub("/target_speed", cbTargetSpeed);

void init_motors(ros::NodeHandle& nh)
{
    //sets up the ESCs and battery info to allow them to spin
    escPair.Init();
    //escPair.Calibrate(); 
    escPair.Arm();

    escPair.Stop();

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

  if(currTime - lastEncoderReport > 50) // 50 ms loop, for now
  {
    lastEncoderReport = currTime;

    int32_t currTicks = encoder.TakeSnapshot();
    int32_t delta = encoder.CalcDelta(); // TODO: convert to linear speed and publish

    enc_val.data = currTicks;
    pub_enc.publish(&enc_val);

    speed_enc.data = delta * METERS_PER_TICK;
    pub_speed.publish(&speed_enc);
  }  
}