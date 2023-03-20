#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ESC.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include <sensor_msgs/BatteryState.h>
#include "encoderController.h"
#include "Adafruit_VL53L0X.h"
#include "constants.h"
#define USE_USBCON

// battery Monitor
Adafruit_INA260 bat_monitor = Adafruit_INA260();

ESC esc1(ESC1_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);
ESC esc2(ESC2_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);


encoderController EC = encoderController();
int encoder_counts=0;

ros::NodeHandle nh;
sensor_msgs::BatteryState bat_msg;
ros::Publisher pub_bat_level("/battery", &bat_msg);

std_msgs::Float32 enc_val, rf_front_val, rf_back_val;
std_msgs::Bool man_override;
std_msgs::Int32 speaker_val;
ros::Publisher pub_enc("/encoder", &enc_val);
ros::Publisher pub_rf_front("/rangefinder/front", &rf_front_val);
ros::Publisher pub_rf_back("/rangefinder/back", &rf_back_val);
ros::Publisher pub_man_override("/manual_override", &man_override);
ros::Publisher pub_speakers("/play_sound", &speaker_val);


Adafruit_VL53L0X sensor = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
unsigned long range_timer;
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

    // setup_rangefinder();
    // pinMode(RADIO_OVERRIDE_PIN, INPUT);
    // Serial.begin(9600); // when running robot.launch, comment this out
}

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

void setSpeed(int percent){
  int value = percent*1;
  setThrottle(1500+value);

}

void setThrottle(int throttle){
    //THESE GET THE BATTERY INFO AND ARE NEEDED TO MAKE MOTOR SPIN ^^^^
    bat_msg.voltage = bat_monitor.readBusVoltage();
    bat_msg.current = bat_monitor.readCurrent();
    pub_bat_level.publish(&bat_msg);

    // Sets the speed of the motors with a given input
    esc1.speed(throttle);
    esc2.speed(throttle);
    Serial.println("Driving at ");
    Serial.print(throttle);
    Serial.println("");
}




void drive_rpm(double target_speed){
    float pid_speed = EC.pid_effort_rpm(target_speed);
    // setSpeed(pid_speed);
    Serial.println("Driving at PID ");
    Serial.print(pid_speed);
    Serial.println("");
}

void drive_forward_inches(long inches){
    // if(dist_traveled >= inches){
    //     setSpeed(1500);
    // }else{
    //     setSpeed(1550);
    // }
}

void publishEncCounts(){
    int ecounts = EC.get_encoder_counts();
    char result[20];
    enc_val.data = ecounts;
    dtostrf(ecounts, 20, 5, result);
    pub_enc.publish(&enc_val);
    nh.logwarn(result);
}

void setup() {
    init_motors();
    EC.init();
    //Serial.begin(9600); // when running robot.launch, comment this out
}


void loop() {
    setSpeed(-100);
    publishEncCounts();
    rangefinder();
    override_was_active = true;
    sound_regulator++;
    nh.spinOnce();
} 