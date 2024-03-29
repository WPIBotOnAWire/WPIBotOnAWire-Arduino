#pragma once
#include <math.h>
#include "pins_arduino.h"

// Motor Constants
// fullforward = 2000, fullback = 1000
#define MOTOR_FULLFORWARD    1600.0
#define MOTOR_FULLBACK       1400.0
#define MOTOR_STOP           1500.0

#define RF_MAX_DIST           200

// Pins
#define LED_PIN                12
#define ENCODER_PIN1           45
#define ENCODER_PIN2           47
     
#define TRIGGER_PIN_FRONT      21
#define ECHO_PIN_FRONT         20
#define TRIGGER_PIN_BACK        3
#define ECHO_PIN_BACK           2

#define ESC1_PIN                8 
#define ESC2_PIN                11

// V These pins probably need to be changed

#define BATTERY_SCL             16
#define BATTERY_SDA             17

// RADIO CONSTANTS

#define RADIO_OVERRIDE_VOLTAGE  4.0
#define RADIO_OVERRIDE_PIN      PIN_A4
#define RADIO_OVERRIDE_THROTTLE PIN_A5
#define RADIO_OVERRIDE_LIGHTS   PIN_A3
#define RADIO_OVERRIDE_SOUND    PIN_A2
#define RADIO_OVERRIDE_DETECT   PIN_A6

// SONAR CONSTANTS

#define USPin1                 PIN_A0
#define USPin2                 PIN_A1

// DISTANCE CONSTANCE

#define APPROACH_DISTANCE       1.0 // meters
#define STOP_DISTANCE           0.25 // meters
#define DISTANCE_CONSTANT       100.0 / 30 .0// multiplying number

// the number of times loop() runs in a second
// might change if more things are added to loop()
#define LOOP_CONSTANT           17

// ENCODER AND PID CONSTANTS
#define WHEEL_RADIUS            0.825/39.37; //inches to m
#define WHEEL_CIRCUMFRANCE      2.0*PI*WHEEL_RADIUS;
#define PPR                     1024.0; // pulses per revolution of the encoder
#define ROTARYSTEPS 1
#define ROTARYMIN -16000
#define ROTARYMAX 16000
#define PIN_IN1 ENCODER_PIN1
#define PIN_IN2 ENCODER_PIN2
