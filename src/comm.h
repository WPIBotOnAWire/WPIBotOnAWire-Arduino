/** 
 * By defining USE_USBCON, ROS will use the USB interface and debugging will be set up on Serial1.
 * If you want to do so, USE_USBCON needs to be defined before including ros.h.
 * 
 * If you comment it out, it's reversed: ROS will use Serial1 and debugging happens on the USB.
 * 
 * Serial1 consists of Pin 0 (RX) and Pin 1 (TX)
 * 
 * Historically, you're better off commenting it out: there can be glitches with the USB,
 * plus it's easier to run the debugging in parallel if the USB is dedicated to debugging.
 */
//#define USE_USBCON 

#ifdef USE_USBCON
  #define DEBUG_SERIAL Serial1 //pins 0/1 on the SAMD21 mini breakout
#else
  #define DEBUG_SERIAL SerialUSB
#endif
