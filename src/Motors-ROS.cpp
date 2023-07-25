#include "Motors-ROS.h"

#include <ESC.h>

#include <std_msgs/Int16.h>

#define Serial SerialUSB

// Motor Constants
// fullforward = 2000, fullback = 1000
#define MOTOR_FULLFORWARD    1600.0
#define MOTOR_FULLBACK       1400.0
#define MOTOR_STOP           1500.0

#define ESC1_PIN                8 
#define ESC2_PIN                11

ESC esc1(ESC1_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);
ESC esc2(ESC2_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);

void stopMotors()
{
    esc1.speed(MOTOR_STOP);
    esc2.speed(MOTOR_STOP);
}

void setThrottle(int throttle)
{
    //updateBat();

    // Sets the speed of the motors with a given input
    esc1.speed(throttle);
    esc2.speed(throttle);
    Serial.println("Driving at ");
    Serial.print(throttle);
    Serial.println("");
}

void setSpeed(int percent)
{
  int value = percent*1 + 1500;
  if (percent == 0)
  {
    stopMotors();
  } 

  else
  {
    setThrottle(value);
  }
}

// callback function for receiving speed commands
void cb_motor(const std_msgs::Int16& msg) 
{
    setSpeed(msg.data);
}

ros::Subscriber<std_msgs::Int16> motor_sub("/motor_speed", cb_motor);

void init_motors(ros::NodeHandle& nh)
{
    //sets up the ESCs and battery info to allow them to spin

    esc1.arm();
    esc2.arm();

    delay(500); //???

    stopMotors();

    //nh.advertise(pub_bat_level);
    //nh.advertise(pub_enc);

    nh.subscribe(motor_sub);

    //bat_monitor.begin();
}


