#include "Motors-ROS.h"

#include <ESC.h>
#include "encoder.h"

#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

#include "esc-samd21.h"


#define Serial SerialUSB

// Motor Constants
// fullforward = 2000, fullback = 1000
#define MOTOR_FULLFORWARD    1600.0
#define MOTOR_FULLBACK       1400.0
#define MOTOR_STOP           1500.0

#define ESC1_PIN                6
#define ESC2_PIN                7

// ENCODER AND PID CONSTANTS
#define ENCODER_PIN1           2
#define ENCODER_PIN2           3

#define WHEEL_RADIUS            0.0210  // meters // 0.825/39.37; //inches to m
#define WHEEL_CIRCUMFRANCE      0.0132  // meters // 2.0*PI*WHEEL_RADIUS;
#define PPR                     1024.0  // pulses per revolution of the encoder
#define METERS_PER_TICK         0.0001286 // meters/tick
#define CM_PER_TICK             0.01286 // cm/tick // maybe use this?

QuadEncoder<ENCODER_PIN1,ENCODER_PIN2> encoder; 

std_msgs::Int32 enc_val;
ros::Publisher pub_enc("/encoder", &enc_val);

ESC esc1(ESC1_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);
ESC esc2(ESC2_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);

ESCDirect escPair;

// void stopMotors()
// {
//     esc1.speed(MOTOR_STOP);
//     esc2.speed(MOTOR_STOP);
// }

// void setThrottle(int throttle)
// {
//     //updateBat();

//     // Sets the speed of the motors with a given input
//     esc1.speed(throttle);
//     esc2.speed(throttle);

//     Serial.println("Driving at ");
//     Serial.print(throttle);
//     Serial.println("");
// }

// void setSpeed(int percent)
// {
//   int value = percent*1 + 1500;
//   if (percent == 0)
//   {
//     stopMotors();
//   } 

//   else
//   {
//     setThrottle(value);
//   }
// }

// callback function for receiving speed commands
void cb_motor(const std_msgs::Int16& msg) 
{
    escPair.SetSpeed(msg.data);
}

ros::Subscriber<std_msgs::Int16> motor_sub("/motor_speed", cb_motor);

void init_motors(ros::NodeHandle& nh)
{
    //sets up the ESCs and battery info to allow them to spin
    escPair.Init();
    escPair.Arm();

    //esc1.arm();
    //esc2.arm();

    //stopMotors();
    escPair.Stop();

    //nh.advertise(pub_bat_level);

    nh.subscribe(motor_sub);

    //bat_monitor.begin();
}

//  encoder stuff
void setup_encoder(ros::NodeHandle& nh)
{
  encoder.Init();
  nh.advertise(pub_enc);

  //   while (!nh.connected()){
  //   nh.spinOnce();
  // }
  // while (! Serial);
  // Serial.println("LimitedRotator example for the RotaryEncoder library.");
  // encoder.setPosition(10); // start with the value of 10. //Why???
}

void processEncoders(void)
{
  static uint32_t lastEncoderReport = 0;
  uint32_t currTime = millis();

  if(currTime - lastEncoderReport > 50) // 50 ms loop, for now
  {
    lastEncoderReport = currTime;

    int32_t currTicks = encoder.TakeSnapshot();
    int32_t delta = encoder.CalcDelta();

    enc_val.data = currTicks;
    // Serial.print(enc_val.data);
    // Serial.println();
    pub_enc.publish(&enc_val);
  }  
}


