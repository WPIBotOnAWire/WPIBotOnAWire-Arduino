#include "led-ROS.h"

#include <Arduino.h>
#include <std_msgs/UInt16.h>

volatile uint16_t flashCount = 0;

/**
 * Sets up TC3 for pulsing LEDs. This only works on pin 5!
*/
void setupTC3forLED(void)
{
  // Feed GCLK0 (already enabled) to TCC2 and TC3
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable 
                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK0
                     GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK0 to TCC2 and TC3
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

  TC->CC[0].reg = 18749; // with P = 256, freq = 48^6 / 256 / 18750 = 10 Hz, or 100ms on, 100ms off, etc...
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
    
  // Interrupts
  TC->INTENCLR.reg = 0x3B;           // disable all interrupts
  //TC->INTENSET.bit.OVF = 1;        // hold off on enabling overfollow interrupt -- only needed when flashing
 
  // Enable InterruptVector
  NVIC_EnableIRQ(TC3_IRQn);

  // Enable TC
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // This pipes the TC3 to pin 5 (PA15), but note that it's not enabled until the MUX is enabled in setLED()
  // We use |= to not clobber ESCs!!!
  PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;  

  /**
   * Set up sound on Pin 11 while we're here...
  */

  // GCLK0 is already fed to TCC2 and TC3 above

  // Enable the port multiplexer for digital pin 11 (D11; PA16)
  PORT->Group[g_APinDescription[11].ulPort].PINCFG[g_APinDescription[11].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC2 timer to the port output - port pins are paired odd PMUXO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg = PORT_PMUX_PMUXE_E;

  // Dual slope PWM operation: timers continuously count up to PER register value then down 0
  REG_TCC2_WAVE |= TCC_WAVE_POL(0xF) |           // Reverse the output polarity on all TCC2 outputs
                   TCC_WAVE_WAVEGEN_DSBOTTOM;    // Setup dual slope PWM on TCC0
  while (TCC2->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  // freq = 48MHz / (4 * PER) ==> PER = 48MHz / (4 * freq)
  // 30000 = 400Hz
  REG_TCC2_PER = 30000;      // Set the frequency of the PWM on TCC0 to 50Hz
  while(TCC2->SYNCBUSY.bit.PER);

  REG_TCC2_CCB0 = 15000;   // sets the duty cycle -- 50%    
  while(TCC2->SYNCBUSY.bit.CCB0);

  // Divide the 16MHz signal by 8 giving 2MHz (0.5us) TCC0 timer tick and enable the outputs
  REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV2 |    // Divide GCLK4 by 8
                    TCC_CTRLA_ENABLE;             // Enable the TCC2 output
  while (TCC2->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void setLED(void)
{
  // Enable the port multiplexer for digital pin 5 (D5; PA15). Doing so disconnects the normal pinMode behaviour
  PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.PMUXEN = 1;

  // Turn on the interrupts
  TcCount16* TC = (TcCount16*) TC3;
  TC->INTENSET.bit.OVF = 1;

  // moved to setup -- easier to do there
  // Connect the TC3 timer to the port output - port pins are paired odd PMUXO and even PMUXE
  // Note that we |= so as not to clobber the ESCs!
  //PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;  

  flashCount = 100;
}

void clearLED(void)
{
  // Clear the TC3 timer to the port output - port pins are paired odd PMUXO and even PMUXE  
  PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.PMUXEN = 0;

  // Turn off the interrupts
  TcCount16* TC = (TcCount16*) TC3;
  TC->INTENCLR.bit.OVF = 1;

  // No longer clear the MUX -- just disable above
  // Disconnect the TC3 timer from the port output - port pins are paired odd PMUXO and even PMUXE
  // (this is probably not needed, but won't hurt anything)
  //PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg = 0;  

  // Set to output low to ensure 0 voltage (testing showed it only fell to 0.25V without formally setting to output)
  // pinMode(5, OUTPUT);
}

// callback function for receiving LED commands
void cb_LED(const std_msgs::UInt16& msg)
{
  if(msg.data) setLED();  //turn on LEDs
  else clearLED();        //turn off LEDs

  //set the count for how many flashes we want; counted down in the ISR
  //note that because the timer toggles on ovf, the actual flashes will be half the number sent
  flashCount = -1;  
  if(msg.data == 0) flashCount = 0;
}

ros::Subscriber<std_msgs::UInt16> led_sub("/led_cmd", cb_LED);

void initLED(ros::NodeHandle& nh)
{
  setupTC3forLED();
  //clearLED();

  setLED();

  nh.subscribe(led_sub);
}

void TC3_Handler(void)  // Interrupt on overflow
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
  
  if (TC->INTFLAG.bit.OVF == 1)   // An overflow caused the interrupt
  {
    TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
    if(!(flashCount--)) clearLED();
  }
}
