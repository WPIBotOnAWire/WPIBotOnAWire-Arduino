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

void setSpeed(int throttle){
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

void setup() {
    init_motors();
    //Serial.begin(9600); // when running robot.launch, comment this out
}


void loop() {
    setSpeed(1400);
    
    // // restructure encoder count reads
    // encoder_counts = EC.get_encoder_counts();
    // drive_rpm(0);
    // // this.previousEncoderCounts = encoderCounts;
    // //drive_forward_inches(2.0);
    // int throttle = pulseIn(PIN_A6, HIGH);
    // //Serial.print("THROTTLE ");
    // //Serial.println(throttle);
    
    // // Publish Encoder
    // enc_val.data = encoder_counts;
    // pub_enc.publish(&enc_val);

    // // Publish Rangefinders
    // //Front is MB 1043 (mm model)
    // float rf_front_mVoltage = 0, rf_front_mm = 0, rf_front_in = 0, front_avg = 0;
    // for (int j = 0; j < 5; j++){
    //     rf_front_mVoltage = analogRead(USPin1)/1024.0*5.0*1000.0;
    //     rf_front_mm = rf_front_mVoltage * 5.0 / 4.88; //From Datasheet
    //     rf_front_in = (rf_front_mm * 0.0394); //mm to inch conversion factor
    //     front_avg += rf_front_in;
    // }
    // front_avg /= 5;
    // rf_front_val.data = (rf_front_in);
    // pub_rf_front.publish(&rf_front_val);

    // //Back is MB 1040 (in model)
    // float rf_back_mVoltage = 0, rf_back_in = 0, back_avg = 0;
    // for (int i = 0; i < 5; i++){
    //     rf_back_mVoltage = analogRead(USPin2)/1024.0*5.0*1000.0;
    //     rf_back_in = rf_back_mVoltage / 9.8; //From Datasheet
    //     back_avg += rf_back_in;
    // }
    // back_avg /= 5;
    // rf_back_val.data = (rf_back_in);
    // pub_rf_back.publish(&rf_back_val);

    // // Publish Battery Levels
    // bat_msg.voltage = bat_monitor.readBusVoltage();
    // bat_msg.current = bat_monitor.readCurrent();
    // pub_bat_level.publish(&bat_msg);
    // //THESE ARE NEEDED TO MAKE MOTOR SPIN ^^^^
    // rangefinder();

    
    override_was_active = true;
    sound_regulator++;
    nh.spinOnce();
} 