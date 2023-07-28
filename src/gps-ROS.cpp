#include "gps-ROS.h"

#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function

#include <gps.h>

#include <std_msgs/Int32.h>


/*
 * setup serial for the GPS on SERCOM2: 
 * Arduino pin 3 -> sercom 2:1* -> RX
 * Arduino pin 4 -> sercom 2:0* -> TX
 */
Uart gpsSerial (&sercom2, 3, 4, SERCOM_RX_PAD_1, UART_TX_PAD_0);

GPS_EM506 gps(&gpsSerial);

void SERCOM2_Handler()
{
  gpsSerial.IrqHandler();
}

std_msgs::Int32 gps_report;
ros::Publisher pub_gps("/GPS", &gps_report);

void setupGPS(ros::NodeHandle& nh) 
{
  SerialUSB.println(F("setupGPS()"));
  gps.Init();
  
  //assign pins 3 & 4 SERCOM functionality
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(4, PIO_SERCOM_ALT);

  gps.SetActiveNMEAStrings(GGA | RMC);

//   SerialUSB.println(F("Checking for signal."));
//   while(!(gps.CheckSerial() & RMC) && !SerialUSB.available()) {}

//   GPSDatum gpsDatum = gps.GetReading();  
  
  //no longer need RMC
  //gps.SetActiveNMEAStrings(GGA);
  //gps.SetReportTime(2000);
  //gps.SendNMEA("PMTK314,0,5,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");

  nh.advertise(pub_gps);

  SerialUSB.println(F("/setupGPS()"));
}

void processGPS(void) 
{
  if(gps.CheckSerial()) // & GGA) //if we have a GGA
  {
    // this spends time trying to consolidate readings...might want to undo and use ROS constructs
    String reportStr = gps.GetReading().MakeDataString();

    SerialUSB.println(reportStr);  
  }
}