#include "esc-samd21.h"

#include <Arduino.h>

#define MOTOR_UPDATE_MS 20

/**
 * This sets up TCC0 to send "RC-servo" pulses to pin 2. We skip the Arduino Servo library
 * (which is a hack) and use the timers directly. This will produce a much smoother pulse output.
 * 
 * Note that we only set up one channel, as both motors are sent the same command. 
 * Perhaps there will someday be a need to control independently?
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
  
  // Enable the port multiplexer for digital pin 2 (D2; PA14): timer TCC0 output
  PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to the port output - port pins are paired odd PMUXO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXE_F;

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
  REG_TCC0_CCB0 = oMid;       // TCC0 CCB0 - center the servo on D2
  while(TCC0->SYNCBUSY.bit.CCB0);

  // Divide the 16MHz signal by 8 giving 2MHz (0.5us) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV8 |    // Divide GCLK4 by 8
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

/*
 * We don't actually do anything here. The ESC arms by default when the
 * uC powers up (see the Init() function). So we'll just check that 3 
 * seconds has elapsed before we allow commands.
 */
ESCDirect::MOTOR_STATE ESCDirect::Arm(void)
{
    switch(motorState)
    {
        case IDLE:
            if(millis() > 5000)
            {
                motorState = ARMED;
                targetSpeed = currentSpeed = 0;
            }
            break;

        default:
            break;
    }

    return motorState;
}

void ESCDirect::WriteMicroseconds(uint16_t uSec)
{
    // The CCBx register value corresponds to the pulsewidth in microseconds (us)
    REG_TCC0_CCB0 = uSec;       // TCC0 CCB0 - center the servo on D12
    while(TCC0->SYNCBUSY.bit.CCB0);
}

/*
 * Sent a signal to set the ESC speed
 * depends on the calibration minimum and maximum values
 */
ESCDirect::MOTOR_STATE ESCDirect::SetSpeed(int16_t pct)
{
    if(motorState != ARMED) 
    {
        return motorState;
    }

    else targetSpeed = constrain(pct, -100, 100);

    return motorState;
}

ESCDirect::MOTOR_STATE ESCDirect::UpdateMotors(void)
{
    static uint32_t lastMotorUpdate = 0;
    uint32_t currTime = millis();

    if(currTime - lastMotorUpdate > MOTOR_UPDATE_MS) 
    {
        lastMotorUpdate = currTime;

        if(motorState == ARMED) 
        {
            // SerialUSB.print(targetSpeed);
            // SerialUSB.print('\t');
            // SerialUSB.print(currentSpeed);
            // SerialUSB.print('\n');

            if(currentSpeed < targetSpeed) currentSpeed += 1.0;
            if(currentSpeed > targetSpeed) currentSpeed -= 1.0;

            uint16_t pulseUS = oMid + currentSpeed * (oMax - oMin) / 200;
            pulseUS = constrain(pulseUS, oMin, oMax);
            WriteMicroseconds(pulseUS);
        }
    }

    return motorState;
}

void updateMotors(void) {escPair.UpdateMotors();}