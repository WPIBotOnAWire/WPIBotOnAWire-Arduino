#include "tfmini-ROS.h"

#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include <std_msgs/UInt16.h>

#define Serial SerialUSB

std_msgs::UInt16 tfForeCMmsg; //in cm
ros::Publisher pubTFfore("/rangefinder/fore/TF", &tfForeCMmsg);

std_msgs::UInt16 tfAftCMmsg; //in cm
ros::Publisher pubTFaft("/rangefinder/aft/TF", &tfAftCMmsg);

/**
 * Sets up a tfmini on SERCOM5, pins D6 and D7.
 * 
 * TO MAKE THIS WORK, I HAD TO 'DECOMMISSION' SERCOM5 IN THE ARDUINO SETUP!!!
 * 
 * SERCOM5 should _not_ be active on the SAMD21 mini, but it was -- probably just using the Dev board definition
 * for convenience. Anyway, you need to comment out the SERCOM5 references in Variant.cpp
*/

/*
 * setup serial for the TFmini on SERCOM5: 
 * Arduino pin 6 -> sercom 5:2 -> TX
 * Arduino pin 7 -> sercom 5:3 -> RX
 * 
 * Uart(SERCOM *_s, uint8_t _pinRX, uint8_t _pinTX, SercomRXPad _padRX, SercomUartTXPad _padTX);
 */
Uart tfSerialFore (&sercom5, 7, 6, SERCOM_RX_PAD_3, UART_TX_PAD_2);

/*
 * setup serial for the TFmini on SERCOM4: 
 * Arduino pin A1 -> sercom 4:0 -> TX
 * Arduino pin A2 -> sercom 4:1 -> RX
 * 
 * Uart(SERCOM *_s, uint8_t _pinRX, uint8_t _pinTX, SercomRXPad _padRX, SercomUartTXPad _padTX);
 */
Uart tfSerialAft (&sercom4, A2, A1, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void SERCOM5_Handler(void)
{
  tfSerialFore.IrqHandler();
}

void SERCOM4_Handler(void)
{
  tfSerialAft.IrqHandler();
}

void TFmini::setupTFmini(HardwareSerial* ser)
{
    //start the serial
    serial = ser;  // this could be prettier...
    serial->begin(115200);
}

bool TFmini::handleUART(uint8_t b)
{
  bool retVal = false;
  switch(tfIndex)
  {
    case 0:
      if(b == 0x59) {tfMiniArray[tfIndex++] = b; checkSum = 0x59;} //first byte must be 0x59
      break;
    case 1:
      if(b == 0x59) {tfMiniArray[tfIndex++] = b; checkSum += 0x59;}
      else tfIndex = 0; //didn't get the second 0x59 byte, so restart
      break;
    case 8: //checksum
      if(b == checkSum) //correct end byte, so process
      {
        retVal = true;
        tfIndex = 0;
      } 
      else tfIndex = 0; //didn't get the correct checkSum byte, so restart
      break;
    case 9:
      Serial.println("Something is very wrong!"); //PANIC
      break;
    default:
      tfMiniArray[tfIndex++] = b;
      checkSum += b;
  }

  return retVal;
}

bool TFmini::getDistance(uint16_t& distance)
{
    bool retVal = false;
    while(serial->available())
    {
        char c = serial->read();
        if(handleUART(c))
        {
            memcpy(&distance, &tfMiniArray[2], 2);
            retVal = true;
        }
    }

    return retVal;
}

TFmini tfFore;
TFmini tfAft;

void processTFminis(void)
{
  uint16_t foreDistance = 0;
  if(tfFore.getDistance(foreDistance))
  {
    tfForeCMmsg.data = foreDistance;
    pubTFfore.publish(&tfForeCMmsg);
  }

  uint16_t aftDistance = 0;
  if(tfAft.getDistance(aftDistance))
  {
    tfAftCMmsg.data = aftDistance;
    pubTFaft.publish(&tfAftCMmsg);
  }
}

void setupTFminis(ros::NodeHandle& nh)
{   
    tfFore.setupTFmini(&tfSerialFore);

    //then we have to set the pads
    //assign pins 6 & 7 SERCOM functionality
    pinPeripheral(6, PIO_SERCOM);
    pinPeripheral(7, PIO_SERCOM);

    nh.advertise(pubTFfore);

    tfFore.setOutputRate(10);

    tfAft.setupTFmini(&tfSerialAft);

    //then we have to set the pads
    //assign pins A1 & A2 SERCOM functionality
    pinPeripheral(A1, PIO_SERCOM);
    pinPeripheral(A2, PIO_SERCOM);

    nh.advertise(pubTFaft);

    tfFore.setOutputRate(10);
}

void TFmini::setOutputRate(uint16_t rateHz)
{
    uint8_t lowByte = rateHz;
    uint8_t highByte = rateHz >> 8;

    serial->write((uint8_t)0x5A);
    serial->write((uint8_t)0x06);
    serial->write((uint8_t)0x03);
    serial->write((uint8_t)lowByte);
    serial->write((uint8_t)highByte);
    serial->write((uint8_t)(0x5A + 0x06 + 0x03 + lowByte + highByte));
}