#include "led-ROS.h"

#include <Arduino.h>

/**
 * Sets up TC3 for pulsing LEDs.
*/
void initLED(void)
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

  // Enable the port multiplexer for digital pin 5 (D5; PA15): timer TC3 output
  PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.PMUXEN = 1;
  
  // Connect the TC3 timer to the port output - port pins are paired odd PMUXO and even PMUXE
  PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg = PORT_PMUX_PMUXO_E;
  
  // Enable TC
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}