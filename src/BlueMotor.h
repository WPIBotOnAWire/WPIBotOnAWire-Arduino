#include "Arduino.h"

class BlueMotor 
{
public:
  BlueMotor(){
    motor_setup();
  }

  void moveTo(long position); 

  void setEffort(int effort)
  {
    if (effort < 0) {
      setEffortHidden(-effort, true);
    }
    else {
      setEffortHidden(effort, false);
    }
  }

  

  void motor_setup()
  {
    // Movement setup
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);

    setEffort(0); // set up motors and be sure they're stopped
  }

private:
  const int tolerance = 3;
  const int PWMOutPin = 11;
  const int AIN2 = 4;
  const int AIN1 = 13;
  const int ENCA = 0;
  const int ENCB = 1;
  void setEffortHidden(int effort, bool clockwise)
  {
    if (clockwise) {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }
    else {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    }
    int value = constrain(effort, 0, 400);
    analogWrite(PWMOutPin, value);
  }
  
};
