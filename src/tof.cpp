#include "tof.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
void tof::setup() {
  Serial.begin(9600);
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
}

// return distance in cm
float tof::distance() {
  VL53L0X_RangingMeasurementData_t measure;
  Serial.print("Reading a measurement... ");
  float distance = 0;

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
  distance = measure.RangeMilliMeter*10.0;
    Serial.print("Distance (cm): "); Serial.println(distance);
  } else {
    Serial.println(" out of range ");
    distance = -1.0;
  }

  delay(100);
  return distance;
}