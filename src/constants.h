#pragma once
#include <math.h>
#include "pins_arduino.h"


#define RF_MAX_DIST           200

// Pins
#define LED_PIN                12

#define TRIGGER_PIN_FRONT      21
#define ECHO_PIN_FRONT         20
#define TRIGGER_PIN_BACK        3
#define ECHO_PIN_BACK           2

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

