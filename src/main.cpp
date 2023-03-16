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

#include "constants.h"

#define USE_USBCON

Adafruit_INA260 bat_monitor = Adafruit_INA260();

ESC esc1(ESC1_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);
ESC esc2(ESC2_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);

ros::NodeHandle nh;
sensor_msgs::BatteryState bat_msg;
ros::Publisher pub_bat_level("/battery", &bat_msg);

int sound_regulator = 0;

bool override_was_active = false;

void init_motors(){
    //sets up the ESCs and battery info to allow them to spin
    //ESCs need battery to spin. 2021-22 team was a bunch of drone bros and picked this garbage
    esc1.arm();
    esc2.arm();

    delay(500);

    esc1.speed(MOTOR_STOP);
    esc2.speed(MOTOR_STOP);

    nh.initNode();
    nh.advertise(pub_bat_level);
    bat_monitor.begin();
}

void setSpeed(int throttle){
    //THESE GET THE BATTERY INFO AND ARE NEEDED TO MAKE MOTOR SPIN ^^^^
    bat_msg.voltage = bat_monitor.readBusVoltage();
    bat_msg.current = bat_monitor.readCurrent();
    pub_bat_level.publish(&bat_msg);

    // Sets the speed of the motors with a given input
    esc1.speed(throttle);
    esc2.speed(throttle);
}

void setup() {
    init_motors();
    //Serial.begin(9600); // when running robot.launch, comment this out
}


void loop() {
    setSpeed(1400);
    override_was_active = true;
    sound_regulator++;
    nh.spinOnce();
}