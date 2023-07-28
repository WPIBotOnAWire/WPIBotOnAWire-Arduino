/*
 * For managing quadrature encoders
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include <Arduino.h>

class Encoder
{
public:
  virtual int32_t TakeSnapshot(void) = 0;
  virtual int32_t CalcDelta(void) = 0;

  virtual void Init(void) {}
};

template <uint8_t A, uint8_t B> class QuadEncoder : public Encoder
{
private:
  static volatile int32_t currTicks; //the current encoder value

  volatile int32_t snapTicks = 0; //frozen "snapshot" for speed calculations
  volatile int32_t prevTicks = 0; //previous value for calculating speed

  static void ProcessA(void)
  {
    //could speed up with direct register call, but we'll use digitalRead for now
    if (digitalRead(A) == digitalRead(B)) currTicks++;
    else currTicks--;
  }

  static void ProcessB(void) 
  {
    //could speed up with direct register call, but we'll use digitalRead for now
    if (digitalRead(A) == digitalRead(B)) currTicks--;
    else currTicks++;
  }

public:
  QuadEncoder(void) {}

  virtual void Init(void)
  {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("QuadEncoder<>::Init");
#endif

    AttachInterrupts();

#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("/QuadEncoder<>::Init");
#endif
  }

  void AttachInterrupts(void)
  {
    currTicks = 0;
    attachInterrupt(A, ProcessA, CHANGE);
    attachInterrupt(B, ProcessB, CHANGE);
  }

  int32_t TakeSnapshot(void) //volatile
  {
    noInterrupts();
    snapTicks = currTicks;
    interrupts();
    return snapTicks;
  }

  int32_t CalcDelta(void) //volatile
  {
    noInterrupts();
    int32_t delta = snapTicks - prevTicks;
    prevTicks = snapTicks;
    interrupts();

    return delta;
  }
};

template <uint8_t A, uint8_t B> volatile int32_t QuadEncoder<A,B>::currTicks = 0;

#endif
