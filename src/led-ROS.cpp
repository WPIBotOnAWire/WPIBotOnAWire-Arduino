#include "led-ROS.h"

#include <Arduino.h>
#include <std_msgs/UInt16.h>

volatile uint16_t flashCount = 0;

/**
 * Sets up TC3 for pulsing LEDs. This only works on pin 5!
*/
void setupTC3forLED(void)
{
  // Feed GCLK0 (already enabled) to TCC2 (and TC3)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable 
                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK0
                     GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK4 to TCC2 and TC3
  while (GCLK->STATUS.bit.SYNCBUSY) {};           // Wait for synchronization

  // The type cast must fit with the selected timer mode (defaults to 16-bit)
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
  
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;    // Disable TC
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 

  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits (defaults to 16-bit, but why not?)
  while (TC->STATUS.bit.SYNCBUSY == 1);     // wait for sync

  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC mode to TOGGLE on OVF
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;   // Set prescaler
  while (TC->STATUS.bit.SYNCBUSY == 1);         // wait for sync

  TC->CC[0].reg = 37499; // with P = 256, freq = 48^6 / 256 / 37500 = 5 Hz, or 200ms on, 200ms off, etc...
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
    
  // Interrupts
  TC->INTENCLR.reg = 0x3B;           // disable all interrupts
  TC->INTENSET.bit.OVF = 1;          // enable overfollow interrupt
 
  // Enable InterruptVector
  NVIC_EnableIRQ(TC3_IRQn);

  // Enable TC
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void setLED(void)
{
  // Enable the port multiplexer for digital pin 5 (D5; PA15). Doing so disconnects the normal pinMode behaviour
  PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.PMUXEN = 1;

  // Connect the TC3 timer to the port output - port pins are paired odd PMUXO and even PMUXE
  PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg = PORT_PMUX_PMUXO_E;  
}

void clearLED(void)
{
  // Clear the TC3 timer to the port output - port pins are paired odd PMUXO and even PMUXE  
  PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.PMUXEN = 0;

  // Disconnect the TC3 timer from the port output - port pins are paired odd PMUXO and even PMUXE
  // (this is probably not needed, but won't hurt anything)
  PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg = 0;  

  // Set to output low to ensure 0 voltage (testing showed it only fell to 0.25V without)
  pinMode(5, OUTPUT);
}

// callback function for receiving LED commands
void cb_LED(const std_msgs::UInt16& msg)
{
  if(msg.data) setLED();  //turn on LEDs
  else clearLED();        //turn off LEDs

  //set the count for how many flashes we want; counted down in the ISR
  //note that because the timer toggles on ovf, the flashes will be half the number sent
  flashCount = msg.data;  
}

ros::Subscriber<std_msgs::UInt16> led_sub("/led_cmd", cb_LED);

void initLED(ros::NodeHandle& nh)
{
  setupTC3forLED();
  clearLED();

  nh.subscribe(led_sub);
}

void TC3_Handler()  // Interrupt on overflow
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
  
  if (TC->INTFLAG.bit.OVF == 1)   // An overflow caused the interrupt
  {
    TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
    if(!(--flashCount)) clearLED();
  }
}
