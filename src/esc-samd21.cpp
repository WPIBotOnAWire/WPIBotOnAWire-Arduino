#include "esc-samd21.h"

#include <Arduino.h>

// Motor Constants -- TODO: pass to constructor
// fullforward = 2000, fullback = 1000
#define MOTOR_FULLFORWARD    2000
#define MOTOR_FULLBACK       1000
#define MOTOR_STOP           1500

/**
 * This sets up TCC0 to send "RC-servo" pulses to pins 10 and 13. We skip the Arduino Servo library
 * (which is a hack) and use the timers directly. This will produce a much smoother pulse output.
 * 
 * Note that we set up two channels, one for each motor, which is somewhat redundant, as both
 * motors are sent the same command. Perhaps there will someday be a need to control independently?
*/
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

  // Enable the port multiplexer for digital pin 13 (D13; PA17): timer TCC0 output
  PORT->Group[g_APinDescription[13].ulPort].PINCFG[g_APinDescription[13].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to the port output - port pins are paired odd PMUXO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[13].ulPort].PMUX[g_APinDescription[13].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;
  
  // Enable the port multiplexer for digital pin 10 (D10; PA18): timer TCC0 output
  PORT->Group[g_APinDescription[10].ulPort].PINCFG[g_APinDescription[10].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to the port output - port pins are paired odd PMUXO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[10].ulPort].PMUX[g_APinDescription[10].ulPin >> 1].reg = PORT_PMUX_PMUXE_F;
  
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
  REG_TCC0_CCB3 = oArm;       // TCC0 CCB3 - center the servo on D13
  while(TCC0->SYNCBUSY.bit.CCB3);

  // The CCBx register value corresponds to the pulsewidth in microseconds (us)
  REG_TCC0_CCB2 = oArm;       // TCC0 CCB0 - center the servo on D12
  while(TCC0->SYNCBUSY.bit.CCB2);

  // Divide the 16MHz signal by 8 giving 2MHz (0.5us) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV8 |    // Divide GCLK4 by 8
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

/*
 * Sent a signal to Arm the ESC
 * depends on the Arming value from the constructor
 */
void ESCDirect::Arm(void)
{
	WriteMicroseconds(oArm);
    delay(500); //might need to make non-blocking
}

void ESCDirect::WriteMicroseconds(uint16_t uSec)
{
    // The CCBx register value corresponds to the pulsewidth in microseconds (us)
    REG_TCC0_CCB3 = uSec;       // TCC0 CCB3 - center the servo on D13
    while(TCC0->SYNCBUSY.bit.CCB3);

    // The CCBx register value corresponds to the pulsewidth in microseconds (us)
    REG_TCC0_CCB2 = uSec;       // TCC0 CCB0 - center the servo on D12
    while(TCC0->SYNCBUSY.bit.CCB2);
}

/*
 * Sent a signal to set the ESC speed
 * depends on the calibration minimum and maximum values
 */
void ESCDirect::SetSpeed(int16_t pct)
{
    if(pct == 0)
    {
        Stop();
        return;
    }

    uint16_t speed = oMid + pct * (oMax - oMin) / 200;
	uint16_t pulseUS = constrain(speed, oMin, oMax);
	WriteMicroseconds(pulseUS);
}

/*
 * Calibrate the maximum and minimum PWM signal the ESC is expecting
 * depends on the outputMin, outputMax values from the constructor
 */
void ESCDirect::Calibrate(void)
{
	WriteMicroseconds(oMax);
    delay(calibrationDelay);
    WriteMicroseconds(oMin);
	delay(calibrationDelay);
	Arm();
}
