#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ESC.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include <sensor_msgs/BatteryState.h>
#include "Adafruit_VL53L0X.h"
#include "constants.h"
#include <RotaryEncoder.h>
#define USE_USBCON
#define LOX1 0x30
#define LOX2 0x31
#define LOX3 0x32

// Shutdown pins for VL53L0X (XSHUT)
#define XS_LOX1 30
#define XS_LOX2 31
#define XS_LOX3 32

// Objects for the ToF sensor
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// Measurement variables
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

double sensor1, sensor2, sensor3;

// Process to assign new I2C addresses
void setID() {
  // RESET procedure
  digitalWrite(XS_LOX1, LOW);
  digitalWrite(XS_LOX2, LOW);
  digitalWrite(XS_LOX3, LOW);
  delay(10);
  digitalWrite(XS_LOX1, HIGH);
  digitalWrite(XS_LOX2, HIGH);
  digitalWrite(XS_LOX3, HIGH);

  // Activate L0X1 and reset other sensors
  digitalWrite(XS_LOX1, HIGH);
  digitalWrite(XS_LOX2, LOW);
  digitalWrite(XS_LOX3, LOW);

  // Initialize LOX1
  if (!lox1.begin(LOX1)) {
    Serial.println(F("Failed to boot LOX1 VL53L0X"));
    while (1);
  }

  // Initialize LOX2
  digitalWrite(XS_LOX2, HIGH);
  delay(10);

  if (!lox2.begin(LOX2)) {
    Serial.println(F("Failed to boot LOX2 VL53L0X"));
    while (1);
  }
  // Initialize LOX3
  digitalWrite(XS_LOX3, HIGH);
  delay(10);

  if (!lox3.begin(LOX3)) {
    Serial.println(F("Failed to boot LOX3 VL53L0X"));
    while (1);
  }
}

void readLOX() {
  lox1.rangingTest(&measure1, false); // pass in 'true' to get data print
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);

  // Print reading from LOX1
  Serial.print("LOX1: ");
  if (measure1.RangeStatus != 4) {   // if not out of range
    sensor1 = measure1.RangeMilliMeter;
    Serial.print(sensor1);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }

  Serial.print(" ");

  // Print reading from LOX2
  Serial.print("LOX2: ");
  if (measure2.RangeStatus != 4) {   // if not out of range
    sensor2 = measure2.RangeMilliMeter;
    Serial.print(sensor2);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }
  Serial.print(" ");

  Serial.print("LOX3: ");
  if (measure3.RangeStatus != 4) {   // if not out of range
    sensor3 = measure3.RangeMilliMeter;
    Serial.print(sensor3);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }
  Serial.println();
}

void sensorConfig() {
  lox1.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
  lox2.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
  lox3.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);

  /* Set desired configurations from the options below:
   *    VL53L0X_SENSE_DEFAULT
        VL53L0X_SENSE_LONG_RANGE
        VL53L0X_SENSE_HIGH_SPEED
        VL53L0X_SENSE_HIGH_ACCURACY
   */
}

void setup() {
  Serial.begin(115200);
  // Wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }
  pinMode(XS_LOX1, OUTPUT);
  pinMode(XS_LOX2, OUTPUT);
  pinMode(XS_LOX3, OUTPUT);

  Serial.println("Starting...");
  setID();
  sensorConfig();
}

void loop() {
  readLOX();
  delay(100);
}