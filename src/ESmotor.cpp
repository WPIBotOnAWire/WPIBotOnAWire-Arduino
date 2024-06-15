#include "ESmotor.h"
#include "robot.h"
#include "math.h"

uint16_t MOTOR_UPDATE_MS = 20U;

void encoderISR(void)
{
    esMotor.handleEncoderISR();
}

/**
 * Set up TCC0 on pin 2 for PWM at 20 kHz.
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
  
  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization


    // Connect the TCC0 timer to the port output - port pins are paired odd PMUXO and even PMUXE
    // F & E specify the timers: e.g., TCC0, TCC1 and TCC2
    PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;



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

  // No pre-scaler; enable and sync
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 // Divide GCLK4 by 1 (no pre-scaler)
                 | TCC_CTRLA_ENABLE;         // Enable the TCC0 output [should be moved to Arm()?]
  while (TCC0->SYNCBUSY.bit.ENABLE);         // Wait for synchronization


  /**
   * Here we set up a 20ms timer that will be used to schedule the speed controller.
   * General clock 4 is piped to TCC1 (divided by 3) above.
   */

  // Single slope PWM operation
  TCC1->WAVE.reg |= TCC_WAVE_WAVEGEN_NFRQ;
  while (TCC1->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // 20000 => 50Hz; w/pre-scaler of 3 above and 16 below, f=48MHz/3/16/(20*1000) = 50Hz
  REG_TCC1_PER = MOTOR_UPDATE_MS * 1000U;
  while(TCC1->SYNCBUSY.bit.PER);

  // Enable InterruptVector
  NVIC_EnableIRQ(TCC1_IRQn);

  // And set interrupt enable for overflow
  TCC1->INTENSET.reg = TCC_INTENSET_OVF;

  // Divide the 16MHz signal by 16 giving 3MHz TCC1 timer
  REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV16   // Divide GCLK4 by 16
                 | TCC_CTRLA_ENABLE;            // and enable
  while (TCC1->SYNCBUSY.bit.ENABLE);            // Wait for synchronization

  // Set direction pin as output
  pinMode(directionPin, OUTPUT);  

  // Set encoder pin as input, set up interrupt
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, CHANGE);
}

/*
 * Arm the motor by engaging the TCC0 waveform on pin 2.
 */
ESMotor::MOTOR_STATE ESMotor::Arm(void)
{
    switch(motorState)
    {
        case IDLE:
        case OVERRIDE:
            DEBUG_SERIAL.println("Arming motors.");
            motorState = ARMED;
            targetSpeed = currentSetPoint = 0;
            sumError = 0;

            // Enable the port multiplexer for digital pin 2 (D2; PA14): timer TCC0 output
            PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;

            break;

        default:
            break;
    }

    return motorState;
}


ESMotor::MOTOR_STATE ESMotor::Disarm(void)
{
    DEBUG_SERIAL.println("Disarming motors.");

    // Enable the port multiplexer for digital pin 2 (D2; PA14): timer TCC0 output
    PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 0;

    motorState = IDLE;

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
        /**
         * Convert from meters/second to ticks/interval
         */
        targetSpeed = speedMPS / 0.041;
    }

    return motorState;
}

/**
 * Executes a simple PI controller.
 */
ESMotor::MOTOR_STATE ESMotor::UpdateMotors(void)
{
    static uint32_t lastMotorUpdate = 0;
    uint32_t currTime = millis();

    if(readyToPID)
    {
        lastMotorUpdate = currTime; // holdover from prev version; in case I want to check/test
        // DEBUG_SERIAL.print(lastMotorUpdate);
        // DEBUG_SERIAL.print('\t');
        // DEBUG_SERIAL.println(motorState);
        int16_t speed = CalcEncoderSpeed();

        if(motorState == ARMED) 
        {
            // this does the ramping of the motor to avoid jerk
            if(currentSetPoint < targetSpeed) currentSetPoint += 1.0;
            if(currentSetPoint > targetSpeed) currentSetPoint -= 1.0;



            int16_t error = currentSetPoint - speed;
            if(abs(sumError) < integralCap) sumError += error;

            float effort = FeedForward(currentSetPoint) + Kp * error + Ki * sumError;
            SetEffort(effort);

#ifdef __MOTOR_DEBUG__
            DEBUG_SERIAL.print(FeedForward(currentSetPoint));
            DEBUG_SERIAL.print('\t');
            DEBUG_SERIAL.print(effort);
            DEBUG_SERIAL.print('\t');
            DEBUG_SERIAL.print(targetSpeed);
            DEBUG_SERIAL.print('\t');
            DEBUG_SERIAL.print(currentSetPoint);
            DEBUG_SERIAL.print('\t');
#endif
        }

#ifdef __MOTOR_DEBUG__
            DEBUG_SERIAL.print(speed);
            DEBUG_SERIAL.print('\n');
#endif

        // else if(motorState == OVERRIDE)
        // {
        //     WriteMicroseconds(pulseUS);
        // }

        readyToPID = 0;
    }

    return motorState;
}

/**
 * Sets the effort on a scale of [-400, 400]
 */
void ESMotor::SetEffort(int16_t match)
{
    if(match >=0) 
    {
        direction = 1;
        digitalWrite(directionPin, HIGH);
    }
    else 
    { 
        direction = -1;
        digitalWrite(directionPin, LOW); 
        match = -match;
    }

    if(match > 400) match = 400;
    
    REG_TCC0_CCB0 = match;       // TCC0_CCB0 - sets the compare match value on D2
    while(TCC0->SYNCBUSY.bit.CCB0) {}
}

void TCC1_Handler() 
{
  if (TCC1->INTFLAG.bit.OVF) //Test if an OVF-Interrupt has occured
  {
    TCC1->INTFLAG.bit.OVF = 1;  //Clear the Interrupt-Flag
    esMotor.TakeSnapshot();
  }
}
