#include "gps-ROS.h"

#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function

#include <gps.h>

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
