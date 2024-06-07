#include "ESmotor.h"

#define MOTOR_UPDATE_MS 20

/**
 * Set up TCC0 on pin 2. 20 kHz.
 * 
 * Note that we only set up one channel, as both motors are sent the same command. 
 * Perhaps there will someday be a need to control independently?
 * 
 * PLL (48MHz) /3 -> GLCK4 (16MHz) / 1 -> TCC0(16MHz)
 * Dual slope w/TOP = 400 -> 20kHz [16MHz / (400*2) = 20kHz]
*/
void ESMotor::Init(void)
{
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
//                     GCLK_CLKCTRL_GEN_GCLK0 |     // use GCLK0 directly?
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  // Enable the port multiplexer for digital pin 2 (D2; PA14): timer TCC0 output
  PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to the port output - port pins are paired odd PMUXO and even PMUXE
  // F & E specify the timers: e.g., TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXE_F;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers continuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |           // Reverse the output polarity on all TCC0 outputs (?)
                   TCC_WAVE_WAVEGEN_DSBOTTOM;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  // 400 = 20kHz
  REG_TCC0_PER = 400;
  while(TCC0->SYNCBUSY.bit.PER);

  // The CCBx sets the duty cycle 
  REG_TCC0_CCB0 = 0;       
  while(TCC0->SYNCBUSY.bit.CCB0);

  // Divide the 16MHz signal by 8 giving 16MHz (0.5us) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 8
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output [should be moved to Arm()?]
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

/*
 * We don't actually do anything here. The ESC arms by default when the
 * uC powers up (see the Init() function). So we'll just check that N 
 * seconds has elapsed before we allow commands.
 */
ESMotor::MOTOR_STATE ESMotor::Arm(void)
{
    switch(motorState)
    {
        case IDLE:
        case OVERRIDE:
            if(millis() > 6000)
            {
                motorState = ARMED;
                targetSpeed = currentSetPoint = 0;
            }
            break;

        default:
            break;
    }

    return motorState;
}

/*
 * 
 */
ESMotor::MOTOR_STATE ESMotor::SetTargetSpeedMetersPerSecond(float speedMPS)
{
    if(motorState != ARMED) 
    {
        return motorState;
        DEBUG_SERIAL.println("UNARMED!");
    }

    else 
    {
        float pct = speedMPS * 25; // totally made up number... 
        targetSpeed = constrain(pct, -100, 100);
    }

    // I had put this in for testing, but now I don't think it's supposed
    // to be here -- motors are controlled through updateMotors()
    // uint16_t pulseUS = oMid + targetSpeed * (oMax - oMin) / 200;
    // pulseUS = constrain(pulseUS, oMin, oMax);
    // WriteMicroseconds(pulseUS);

    return motorState;
}

ESMotor::MOTOR_STATE ESMotor::UpdateMotors(void)
{
    static uint32_t lastMotorUpdate = 0;
    uint32_t currTime = millis();

    if(currTime - lastMotorUpdate > MOTOR_UPDATE_MS) 
    {
        lastMotorUpdate = currTime;

        if(motorState == ARMED) 
        {
            // DEBUG_SERIAL.print(targetSpeed);
            // DEBUG_SERIAL.print('\t');
            // DEBUG_SERIAL.print(currentSpeed);
            // DEBUG_SERIAL.print('\n');

            // this does the ramping of the motor to avoid jerk
            if(currentSetPoint < targetSpeed) currentSetPoint += 1.0;
            if(currentSetPoint > targetSpeed) currentSetPoint -= 1.0;

            // uint16_t pulseUS = oMid + currentSetPoint * (oMax - oMin) / 200;
            // pulseUS = constrain(pulseUS, oMin, oMax);
            // DEBUG_SERIAL.println(pulseUS);
            // WriteMicroseconds(pulseUS);
        }

        // else if(motorState == OVERRIDE)
        // {
        //     WriteMicroseconds(pulseUS);
        // }
    }

    return motorState;
}

void updateMotors(void) {esMotor.UpdateMotors();}