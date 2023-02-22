#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ESC.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include <sensor_msgs/BatteryState.h>
#include "Adafruit_VL53L0X.h"

#include "constants.h"

#define USE_USBCON

Adafruit_INA260 bat_monitor = Adafruit_INA260();

ESC esc1(ESC1_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);
ESC esc2(ESC2_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);

Encoder encoder(ENCODER_PIN1, ENCODER_PIN2);

ros::NodeHandle nh;
std_msgs::Float32 enc_val, rf_front_val, rf_back_val;
std_msgs::Bool man_override;
std_msgs::Int32 speaker_val;
sensor_msgs::BatteryState bat_msg;
ros::Publisher pub_enc("/encoder", &enc_val);
ros::Publisher pub_rf_front("/rangefinder/front", &rf_front_val);
ros::Publisher pub_rf_back("/rangefinder/back", &rf_back_val);
ros::Publisher pub_bat_level("/battery", &bat_msg);
ros::Publisher pub_man_override("/manual_override", &man_override);
ros::Publisher pub_speakers("/play_sound", &speaker_val);

Adafruit_VL53L0X sensor = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
unsigned long range_timer;
int sound_regulator = 0;
bool override_was_active = false;

void setup_rangefinder(){
  nh.getHardware()->setBaud(57600);
  nh.advertise(pub_rf_front);
  // wait controller to be connected
  while (!nh.connected()){
    nh.spinOnce();
  }
  // if initialization failed - write message and freeze
  if (!sensor.begin()) {
    nh.logwarn("Failed to setup VL53L0X sensor");
    while(1);
  }
  nh.loginfo("VL53L0X API serial node started");
  // fill static range message fields
}

void rangefinder(){
    if ((millis()-range_timer) > 50){
    sensor.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        rf_front_val.data = (float) measure.RangeMilliMeter/1000.0f; // convert mm to m
        pub_rf_front.publish(&rf_front_val);
        float p = measure.RangeMilliMeter;
        char result[20];
        dtostrf(p, 20, 5, result);
        nh.logwarn(result);
    } else {
      rf_front_val.data = (float) -999999999;
      pub_rf_front.publish(&rf_front_val);
      nh.logwarn("Out of range"); // if out of range, don't send message
    }
    range_timer =  millis();    
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void cb_led(const std_msgs::Bool &msg) {
    int state = msg.data ? HIGH : LOW;
    digitalWrite(LED_PIN, state);
}

void cb_motor(const std_msgs::Float32 &msg) {
    int speed = mapfloat(msg.data, -1.0, 1.0, MOTOR_FULLBACK, MOTOR_FULLFORWARD);
    esc1.speed(speed);
    esc2.speed(speed);
}

ros::Subscriber<std_msgs::Bool> led_sub("/deterrents/led", &cb_led);
ros::Subscriber<std_msgs::Float32> motor_sub("/motor_speed", &cb_motor);

void setup() {

    esc1.arm();
    esc2.arm();

    delay(500);

    esc1.speed(MOTOR_STOP);
    esc2.speed(MOTOR_STOP);
    nh.initNode();
    setup_rangefinder();
    nh.advertise(pub_bat_level);
    nh.subscribe(motor_sub);
    bat_monitor.begin();
    //Serial.begin(9600); // when running robot.launch, comment this out
}


void setSpeed(int throttle){
    // Sets the speed of the motors with a given input
    esc1.speed(throttle);
    esc2.speed(throttle);
}


void loop() {
    bat_msg.voltage = bat_monitor.readBusVoltage();
    bat_msg.current = bat_monitor.readCurrent();
    pub_bat_level.publish(&bat_msg);
    //THESE ARE NEEDED TO MAKE MOTOR SPIN ^^^^
    rangefinder();
    override_was_active = true;
    sound_regulator++;
    nh.spinOnce();
}