
/**
 * Current list of pins:
 * 
 * 0,1:   Serial1 for Jetson comms (SERCOM0, set up by Arduino library)
 * 2:     Speed control ***CHANGED 11/5 -- used to be D10,13***
 * 3,4:   GPS on SERCOM2*
 * 5:     LED control
 * 6,7:   ToF sensor on SERCOM5
 * 8:     Direction control
 * 9:     Encoder
 * 10-13: FREE (SPI bus)
 * A0:    MaxBotix
 * A1,A2: ToF sensor on SERCOM4
 * A3:    MaxBotix
 * 20,21: Battery monitor on I2C bus (SERCOM3)
*/

#include <Arduino.h>

#include <ros.h>
#include <std_msgs/UInt32.h>

#include "Motors-ROS.h"
#include "battery-ROS.h"
#include "status-ROS.h"

#include "rangefinder-ROS.h"
#include "tfmini-ROS.h"

#include "led-ROS.h"

#include "gps-ROS.h"
#include "position_filter.h"

#include "radio-ROS.h"

#include "wdt_samd21.h"

#include "robot.h"

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
   * 
   * Doing so can be useful for debugging, however, as the program will wait for the Serial
   * Monitor so you can see start up.
  */
  // while(!DEBUG_SERIAL)
  // {
  //   delay(100);
  // }
  
  DEBUG_SERIAL.println("setup()");

  /**
   * The following code _only_ gets called if there has been a watchdog timout.
   * 
   * It initializes the heartbeat (so we know it's alive) and then goes to processing
   * the radio, which should allow the user to drive the robot back home.
  */
  if(wdt_check_reset()) 
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

  /**
   * Here is the typical setup sequence.
  */
  wdt_init(WDT_CONFIG_PER_8K); // 8 second watchdog timeout

  nh.initNode();
  nh.advertise(heartbeat);

  setupRadio(nh);
  init_status(nh);

  setup_rangefinders(nh);
  setupTFminis(nh);

  init_motors(nh);
  //setup_encoder(nh);

  //initBatteryMonitor(nh);
  //setupGPS(nh);
  initLED(nh);
 
  DEBUG_SERIAL.println("/setup");
}

void loop(void) 
{
  /**
   * We send a heartbeat every second and reset the watchdog (which has an 8 second timeout).
  */
  static uint32_t lastPing = 0;
  uint32_t currTime = millis();
  if(currTime - lastPing >= 1000)
  {
    lastPing = currTime;

    heartbeatMsg.data = millis();
    heartbeat.publish( &heartbeatMsg );

    DEBUG_SERIAL.print(millis());
    DEBUG_SERIAL.println("\tHeartbeat.");

    wdt_reset();
  }

  /**
   * Allows simple testing through the debug interface.
  */
  if(DEBUG_SERIAL.available())
  {
    char ch = DEBUG_SERIAL.read();
    if(ch == 'T') 
    {
      robot.SetTeleop();
    }
    if(ch == 'A') 
    {
      robot.SetAuto();
    }
    if(ch == 'D') 
    {
      robot.Disarm();
    }
  }
  
  processRangefinders(); // ultrasonics
  processTFminis();

  updateMotors();

  //processBatteryMonitor();
  processRadio();

  //processGPS();

  robot.loop();

  nh.spinOnce();
}