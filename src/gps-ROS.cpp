#include "gps-ROS.h"

#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function

#include <gps.h>

#include <gps_common/GPSFix.h>
#include <std_msgs/String.h>
//using namespace gps_common;

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

/**
 * Need to figure out the whole gps_common thing...might need to custom build the messages???
 * 
 * In the meantime, I'll just pass a string and move on.
 * 
 * TODO:
 * GPSFix and GPSStatus are stoooopidly big. I'll just get the data I want and pass it.
*/
gps_common::GPSFix gpsFix;
gps_common::GPSStatus gpsStatus;
// ros::Publisher pub_gps_fix("GPSfix", &gpsFix);
// ros::Publisher pub_gps_status("GPSStatus", &gpsStatus);

std_msgs::String gpsNMEA;
ros::Publisher pubNMEA("strNMEA", &gpsNMEA);

void setupGPS(ros::NodeHandle& nh) 
{
  SerialUSB.println(F("setupGPS()"));
  gps.Init();
  
  //assign pins 3 & 4 SERCOM functionality
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(4, PIO_SERCOM_ALT);

  gps.SetActiveNMEAStrings(GGA | RMC);

//   nh.advertise(pub_gps_fix);
//   nh.advertise(pub_gps_status);
    nh.advertise(pubNMEA);

  SerialUSB.println(F("/setupGPS()"));
}

/**
 * Parses an NMEA string and puts data in ROS messages
*/
GPSDatum ParseNMEA_ROS(const String& nmeaStr)
{
    //SerialUSB.println(nmeaStr);
    GPSDatum gpsDatum;
    
    uint16_t length = nmeaStr.length();
    if(length < 3) return 0;
    
    if(nmeaStr[0] != '$') return 0;
    if(nmeaStr[length - 3] != '*') return 0;
    
    uint8_t checksum = GPS::CalcChecksum(nmeaStr.substring(1, length - 3));
    String checksumStr = String(checksum, HEX);
    checksumStr.toUpperCase();
    String checksumNMEA = nmeaStr.substring(length - 2);
    
    if(checksumStr != checksumNMEA) return gpsDatum;
    
    String NMEAtype = GPSDatum::GetNMEASubstring(nmeaStr, 0);
    
    if (NMEAtype == "GPGGA")
    {
        gpsDatum.gpsFix = GPSDatum::GetNMEASubstring(nmeaStr, 6).toInt();
        //if(!gpsDatum.gpsFix) return gpsDatum;
        
        // time
        gpsDatum.NMEAtoTime(GPSDatum::GetNMEASubstring(nmeaStr, 1));
        
        gpsDatum.lat = GPSDatum::ConvertToDMM(GPSDatum::GetNMEASubstring(nmeaStr, 2));
        if(GPSDatum::GetNMEASubstring(nmeaStr, 3) == String('S')) gpsDatum.lat *= -1;
        
        gpsDatum.lon = GPSDatum::ConvertToDMM(GPSDatum::GetNMEASubstring(nmeaStr, 4));
        if(GPSDatum::GetNMEASubstring(nmeaStr, 5) == String('W')) gpsDatum.lon *= -1;
        
        gpsDatum.elevDM = GPSDatum::GetNMEASubstring(nmeaStr, 9).toFloat() * 10;
        
        gpsDatum.source = GGA;

        gpsFix.status.position_source = gpsDatum.gpsFix;
        gpsFix.latitude = GPSDatum::GetNMEASubstring(nmeaStr, 2).toFloat();
        gpsFix.longitude = GPSDatum::GetNMEASubstring(nmeaStr, 4).toFloat();

        gpsNMEA.data = nmeaStr.c_str();
        pubNMEA.publish(&gpsNMEA);

        //pub_gps_fix.publish(&gpsFix);

        return gpsDatum;
    }
    
    if(NMEAtype == "GPRMC")
    {
        if(GPSDatum::GetNMEASubstring(nmeaStr, 2) != String('A')) return gpsDatum;
        //else gpsDatum.gpsFix = 1;
        
        gpsDatum.lat = GPSDatum::ConvertToDMM(GPSDatum::GetNMEASubstring(nmeaStr, 3));
        if(GPSDatum::GetNMEASubstring(nmeaStr, 4) == String('S')) gpsDatum.lat *= -1;
        
        gpsDatum.lon = GPSDatum::ConvertToDMM(GPSDatum::GetNMEASubstring(nmeaStr, 5));
        if(GPSDatum::GetNMEASubstring(nmeaStr, 6) == String('W')) gpsDatum.lon *= -1;
        
        //gpsDatum.speed = GetNMEASubstring(nmeaStr, 7).toFloat();
        
        gpsDatum.NMEAtoTime(GPSDatum::GetNMEASubstring(nmeaStr, 1));
        gpsDatum.NMEAtoDate(GPSDatum::GetNMEASubstring(nmeaStr, 9));
        
        gpsDatum.source = RMC;
        return gpsDatum;
    }
    
    return 0;
}

void processGPS(void) 
{
    String nmeaStr;
    if(gps.CheckSerialRaw(nmeaStr)) // & GGA) //if we have a GGA
    {
        // SerialUSB.println(nmeaStr); 
        ParseNMEA_ROS(nmeaStr); 
    }
}

