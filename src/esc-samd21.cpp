#include "esc-samd21.h"

#include <Arduino.h>

void ESCDirect::Init(void)
{
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for digital pin 13 (D13): timer TCC0 output
  PORT->Group[g_APinDescription[13].ulPort].PINCFG[g_APinDescription[13].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to the port output - port pins are paired odd PMUXO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;
  
  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers continuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |           // Reverse the output polarity on all TCC0 outputs
                   TCC_WAVE_WAVEGEN_DSBOTTOM;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  // 20000 = 50Hz, 10000 = 100Hz, 2500  = 400Hz
  REG_TCC0_PER = 20000;      // Set the frequency of the PWM on TCC0 to 50Hz
  while(TCC0->SYNCBUSY.bit.PER);

  // The CCBx register value corresponds to the pulsewidth in microseconds (us)
  REG_TCC0_CCB3 = 1500;       // TCC0 CCB3 - center the servo on D13
  while(TCC0->SYNCBUSY.bit.CCB3);

  // Divide the 16MHz signal by 8 giving 2MHz (0.5us) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV8 |    // Divide GCLK4 by 8
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization




    // /**
    //  * We use a hack to set up the timer. Namely, by calling analogWrite on pin 6, Arduino will
    //  * set up all the machinery to do PWM on pin 6, as well as pin 4. We can then alter them with a 
    //  * lot less code.
    //  * 
    //  * I'm just trying to get this working, and I'll tidy it all up 'properly' later.
    // */
    
    // analogWrite(6, 0); //use Arduino to set up PWM machinery on TCC0
    // analogWrite(4, 0); //use Arduino to set up PWM machinery on TCC0

    // //We want ~20 ms loops, so change the prescaler and TOP
    // REG_TCC0_CTRLA &= ~TCC_CTRLA_ENABLE;            // Disable the TCC0 output
    // while (TCC0->SYNCBUSY.bit.ENABLE) {}            // Wait for synchronization

    // REG_TCC0_CTRLA = TCC_CTRLA_PRESCSYNC_GCLK | // superfluous, since it's 0
    //                  TCC_CTRLA_PRESCALER_DIV64 |

    //     TCx->COUNT16.CTRLA.reg | TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_NPWM;

    //   REG_TCC0_WAVE               
    
    // TCC_CTRLA_PRESCALER_DIV64 |   // Divide GCLK4 by 8
    //                     TCC_CTRLA_ENABLE;           // Enable the TCC0 output

    // REG_TCC0_CTRLA = TCC_CTRLA_PRESCALER_DIV64 |   // Divide GCLK4 by 8
    //                     TCC_CTRLA_ENABLE;           // Enable the TCC0 output


    // while (TCC0->SYNCBUSY.bit.ENABLE) {}            // Wait for synchronization
 
 
}
