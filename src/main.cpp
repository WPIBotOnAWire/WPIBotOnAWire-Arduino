
/**
 * Current list of pins:
 * 
 * 0,1:   Serial1 for Jetson comms (SERCOM0, set up by Arduino library)
 * 2:     ESC control ***CHANGED 11/5 -- used to be D10,13***
 * 3,4:   GPS on SERCOM2*
 * 5:     LED control
 * 6,7:   ToF sensor on SERCOM5
 * 8,9:   Encoders
 * 10-13: FREE (SPI bus)
 * A0:    MaxBotix
 * A1,A2: ToF sensor on SERCOM4
 * A3:    MaxBotix
 * 20,21: Battery monitor on I2C bus (SERCOM3)
*/

#include <Arduino.h>
#include <ros.h>
//#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include "Motors-ROS.h"
#include "battery-ROS.h"
#include "status-ROS.h"

#include "rangefinder-ROS.h"
#include "led-ROS.h"

#include "gps-ROS.h"
#include "tfmini-ROS.h"

#include "position_filter.h"
#include "radio-ROS.h"

#include "wdt_samd21.h"

ros::NodeHandle nh;

std_msgs::UInt32 heartbeatMsg;
ros::Publisher heartbeat("/heartbeat", &heartbeatMsg);

/*
std_msgs::Float32 rf_front_val, rf_back_val;
std_msgs::Bool man_override;
std_msgs::Int32 speaker_val;
ros::Publisher pub_man_override("/manual_override", &man_override);
ros::Publisher pub_speakers("/play_sound", &speaker_val);
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
  
  DEBUG_SERIAL.println("setup()");

  if(wdt_check_reset()) //watchdog timout!
  {
    nh.initNode();
    nh.advertise(heartbeat);

    setupRadio(nh);
    while(1)
    {
      processRadio();
      static uint32_t lastPing = 0;
      uint32_t currTime = millis();
      if(currTime - lastPing >= 1000)
      {
        lastPing = currTime;

        heartbeatMsg.data = -1;
        heartbeat.publish( &heartbeatMsg );

        DEBUG_SERIAL.println("Watchdog Timeout!");
      }
    }
  }

  wdt_init(WDT_CONFIG_PER_8K); // 8 second watchdog timeout

  nh.initNode();

  nh.advertise(heartbeat);

  setupRadio(nh);
  init_status(nh);
  setup_rangefinders(nh);
  init_motors(nh);
  setup_encoder(nh);
  initBatteryMonitor(nh);
  //setupGPS(nh);
  setupTFminis(nh);
  initLED(nh);
 
  DEBUG_SERIAL.println("/setup");
}

void loop(void) 
{
  static uint32_t lastPing = 0;
  uint32_t currTime = millis();
  if(currTime - lastPing >= 1000)
  {
    lastPing = currTime;

    heartbeatMsg.data = millis();
    heartbeat.publish( &heartbeatMsg );

//    DEBUG_SERIAL.print('\n');
    DEBUG_SERIAL.print(millis());
    DEBUG_SERIAL.println("\tHeartbeat.");

    wdt_reset();
  }
  
  processRangefinders();
  processEncoders();
  updateMotors();
  processBatteryMonitor();
  //processGPS();
  processTFminis();
  processRadio();

  nh.spinOnce();
}