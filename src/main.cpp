/** 
 * By defining USE_USBCON, ROS will use the USB interface and debugging will be set up on Serial1.
 * USE_USBCON needs to be defined before including ros.h.
 * 
 * If you comment it out, it's reversed: ROS will use Serial1 and debugging happens on the USB.
*/
//#define USE_USBCON 

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include "Motors-ROS.h"
// #include "Adafruit_VL53L0X.h"
#include "battery-ROS.h"

#include "rangefinder-ROS.h"
#include "led-ROS.h"

#include "gps-ROS.h"
#include "tfmini-ROS.h"

#ifdef USE_USBCON
  #define DEBUG_SERIAL Serial1 //pins 0/1 on the SAMD21 mini breakout
#else
  #define DEBUG_SERIAL SerialUSB
#endif

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

/*
std_msgs::Float32 rf_front_val, rf_back_val;
std_msgs::Bool man_override;
std_msgs::Int32 speaker_val;
ros::Publisher pub_man_override("/manual_override", &man_override);
ros::Publisher pub_speakers("/play_sound", &speaker_val);

// Adafruit_VL53L0X sensor = Adafruit_VL53L0X();
// VL53L0X_RangingMeasurementData_t measure;
unsigned long range_timer;
int sound_regulator = 0;
bool override_was_active = false;
*/

void setup()
{
  DEBUG_SERIAL.begin(115200);
  /**
   * Be careful when uncommenting the following, as you _must_ connect to the Serial Monitor if
   * you leave them uncommented!
  */
  // while(!DEBUG_SERIAL)
  // {
  //   delay(100);
  // }
  
  DEBUG_SERIAL.println("setup");

  nh.initNode();

  nh.advertise(chatter);

  setup_rangefinders(nh);
  init_motors(nh);
  setup_encoder(nh);
  initBatteryMonitor(nh);
  setupGPS(nh);
  initLED(nh);
  setupTFminis(nh);

    // // pinMode(RADIO_OVERRIDE_PIN, INPUT);
 
  DEBUG_SERIAL.println("/setup");
}
  
// long timer;
char hello[13] = "hello world!";

void loop(void) 
{
  static uint32_t lastPing = 0;
  uint32_t currTime = millis();
  if(currTime - lastPing >= 1000)
  {
    lastPing = currTime;

    str_msg.data = hello;
    chatter.publish( &str_msg );

    DEBUG_SERIAL.print('\n');
    DEBUG_SERIAL.print(millis());
    DEBUG_SERIAL.println("\tJust one ping.");
  }
  
  processRangefinders();
  processEncoders();
  processBatteryMonitor();
  processGPS();
  processTFminis();

    // if(millis()-timer> 1000){
    //   override_was_active = true;
    //   sound_regulator++;
    //   timer = millis();
    // }

  nh.spinOnce();
}