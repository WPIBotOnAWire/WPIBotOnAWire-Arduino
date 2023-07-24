#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ESC.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include <sensor_msgs/BatteryState.h>
#include "Adafruit_VL53L0X.h"
#include "constants.h"
#include <RotaryEncoder.h>

#include "rangefinder.h"

/** 
 * By defining USE_USBCON, ROS will use the USB interface and debugging will be set up on Serial1.
*/
#define USE_USBCON 

#ifdef USE_USBCON
  #define DEBUG_SERIAL Serial1 //pins 0/1 on the SAMD21 mini breakout
#else
  #define DEBUG_SERIAL SerialUSB
#endif

// battery Monitor
Adafruit_INA260 bat_monitor = Adafruit_INA260();

ESC esc1(ESC1_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);
ESC esc2(ESC2_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);


// Setup a RotaryEncoder with 4 steps per latch for the 2 signal input pins:
// RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);
// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);
int encoder_counts=0;

ros::NodeHandle nh;
sensor_msgs::BatteryState bat_msg;
ros::Publisher pub_bat_level("/battery", &bat_msg);

std_msgs::Float32 rf_front_val, rf_back_val;
std_msgs::Bool man_override;
std_msgs::Int32 speaker_val, enc_val;
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

void stopMotors(){
    esc1.speed(MOTOR_STOP);
    esc2.speed(MOTOR_STOP);
}

void updateBat(){
    //THESE GET THE BATTERY INFO AND ARE NEEDED TO MAKE MOTOR SPIN ^^^^
    bat_msg.voltage = bat_monitor.readBusVoltage();
    bat_msg.current = bat_monitor.readCurrent();
    pub_bat_level.publish(&bat_msg);
}


void setThrottle(int throttle){
    updateBat();

    // Sets the speed of the motors with a given input
    esc1.speed(throttle);
    esc2.speed(throttle);
    Serial.println("Driving at ");
    Serial.print(throttle);
    Serial.println("");
}

void setSpeed(int percent){
  int value = percent*1 + 1500;
  if (percent == 0){
    stopMotors();
  } else{
    setThrottle(value);
  }
  

}


// function to control leds in state machine
void cb_led(const std_msgs::Bool &msg) {
    int state = msg.data ? HIGH : LOW;
    digitalWrite(LED_PIN, state);
}

// function to control motors in state machine
void cb_motor(const std_msgs::Int16& msg) {
    //int speed = mapfloat(msg.data, -1.0, 1.0, MOTOR_FULLBACK, MOTOR_FULLFORWARD);
    int speed = msg.data;
    setSpeed(speed);
}

ros::Subscriber<std_msgs::Bool> led_sub("/deterrents/led", cb_led);
ros::Subscriber<std_msgs::Int16> motor_sub("/motor_speed", cb_motor);

void init_motors(){
    //sets up the ESCs and battery info to allow them to spin
    //ESCs need battery to spin. 2021-22 team was a bunch of drone bros and picked this garbage
    esc1.arm();
    esc2.arm();

    delay(500);

    stopMotors();

    nh.advertise(pub_bat_level);
    nh.advertise(pub_enc);
    nh.subscribe(motor_sub);
    bat_monitor.begin();

}

void setup_rangefinder()
{
  nh.getHardware()->setBaud(9600);
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
        float p = measure.RangeMilliMeter /1000.0f; // m
        //char result[20];
        //dtostrf(p, 20, 5, result);
        // nh.logwarn(result);
    } else {
      rf_front_val.data = (float) -999999999;
      pub_rf_front.publish(&rf_front_val);
      // nh.logwarn("Out of range"); // if out of range, don't send message
    }
    range_timer =  millis();    
  }
}

void drive_rpm(double target_speed){
    // float pid_speed = EC.pid_effort_rpm(target_speed);
    // // setSpeed(pid_speed);
    // Serial.println("Driving at PID ");
    // Serial.print(pid_speed);
    // Serial.println("");
}

void drive_forward_meters(long meters){
    // if(dist_traveled >= inches){
    //     setSpeed(1500);
    // }else{
    //     setSpeed(1550);
    // }
}
//  encoder stuff
void setup_encoder(){
    while (!nh.connected()){
    nh.spinOnce();
  }
  while (! Serial);
  Serial.println("LimitedRotator example for the RotaryEncoder library.");
  encoder.setPosition(10 / ROTARYSTEPS); // start with the value of 10.
}

void publishEncCounts(int ecounts){
    enc_val.data = ecounts;
    // Serial.print(enc_val.data);
    // Serial.println();
    pub_enc.publish(&enc_val);
    nh.spinOnce();
}

int lastPos = -1;
int encodercount = 0;
void encoderCounts(){
    encoder.tick();
  // get the current physical position and calc the logical position
  int newPos = encoder.getPosition() * ROTARYSTEPS;
  if (newPos < ROTARYMIN) {
    encoder.setPosition(ROTARYMIN / ROTARYSTEPS);
    newPos = ROTARYMIN;
  } else if (newPos > ROTARYMAX) {
    encoder.setPosition(ROTARYMAX / ROTARYSTEPS);
    newPos = ROTARYMAX;
  } // if
  if (lastPos != newPos) {
    lastPos = newPos;
    encodercount = newPos;
    
  } // if
  publishEncCounts(encodercount);
}


/*
void initAllNodes(){
  
}
*/

void setup()
{
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL)
  {
    delay(100);
  }
  
  DEBUG_SERIAL.println("setup");

  nh.initNode();

    init_motors();
    // setup_rangefinder();
    setup_encoder();
    // pinMode(RADIO_OVERRIDE_PIN, INPUT);
 
  DEBUG_SERIAL.println("/setup");
}
  
long timer;
void loop() {
    if(millis()-timer> 1000){
      updateBat();
      // rangefinder();
      // encoderCounts();
      override_was_active = true;
      sound_regulator++;
      timer = millis();
    }
  nh.spinOnce();
} 
