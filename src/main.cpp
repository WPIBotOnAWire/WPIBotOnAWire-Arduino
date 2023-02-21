#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ESC.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include <sensor_msgs/BatteryState.h>
#include <BlueMotor.h>
#include <Wire.h>
#include "constants.h"
#include <vl53l4cx_class.h>

#define USE_USBCON

ros::NodeHandle nh;

void cb_led(const std_msgs::Bool &msg) {
    int state = msg.data ? HIGH : LOW;
    digitalWrite(LED_PIN, state);
}

ros::Subscriber<std_msgs::Bool> led_sub("/deterrents/led", &cb_led);

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  nh.initNode();
  nh.subscribe(led_sub);
}

void lights_control(int light_switch){
    // light_switch is the pulseIn reading, checks if flipped up or down; down is ~1800-1900, up is ~1000-1100
    if (light_switch < 1500) digitalWrite(LED_PIN, HIGH);
    else digitalWrite(LED_PIN, LOW);
}

void loop()
{
    nh.spinOnce();
}