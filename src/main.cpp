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
#include "encoderController.h"
#include "rangefinder.h"

#define USE_USBCON

// battery Monitor
Adafruit_INA260 bat_monitor = Adafruit_INA260();
sensor_msgs::BatteryState bat_msg;
ros::Publisher pub_bat_level("/battery", &bat_msg);

ESC esc1(ESC1_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);
ESC esc2(ESC2_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);

rangefinder rf;
rosHandler rh = rf.rh;
ros::NodeHandle NH = rh.nh;
encoderController EC = encoderController();
int encoder_counts = 0;
int sound_regulator = 0;
bool override_was_active = false;

void setup()
{

  esc1.arm();
  esc2.arm();

  delay(500);

  esc1.speed(MOTOR_STOP);
  esc2.speed(MOTOR_STOP);
  rf.init();
  NH.advertise(pub_bat_level);
  bat_monitor.begin();
  pinMode(RADIO_OVERRIDE_PIN, INPUT);
  // Serial.begin(9600); // when running robot.launch, comment this out
}

void setSpeed(int throttle)
{
  // Sets the speed of the motors with a given input
  esc1.speed(throttle);
  esc2.speed(throttle);
  Serial.println("Driving at ");
  Serial.print(throttle);
  Serial.println("");
}

void drive_rpm(double target_speed)
{
  float pid_speed = EC.pid_effort_rpm(target_speed);
  // setSpeed(pid_speed);
  Serial.println("Driving at PID ");
  Serial.print(pid_speed);
  Serial.println("");
}

void drive_forward_inches(long inches)
{
  // if(dist_traveled >= inches){
  //     setSpeed(1500);
  // }else{
  //     setSpeed(1550);
  // }
}

void publishBatLevels(float voltage, float current)
{
    bat_msg.voltage = voltage;
    bat_msg.current = current;

    pub_bat_level.publish(&bat_msg);
}


void loop()
{

  // restructure encoder count reads
  encoder_counts = EC.get_encoder_counts();
  drive_rpm(0);
  // this.previousEncoderCounts = encoderCounts;
  // drive_forward_inches(2.0);
  int throttle = pulseIn(PIN_A6, HIGH);
  // Serial.print("THROTTLE ");
  // Serial.println(throttle);

  // Publish Encoder
  rh.publishEncoderCounts(encoder_counts);

  // Publish Rangefinders
  // Front is MB 1043 (mm model)
  float rf_front_mVoltage = 0, rf_front_mm = 0, rf_front_in = 0, front_avg = 0;
  for (int j = 0; j < 5; j++)
  {
    rf_front_mVoltage = analogRead(USPin1) / 1024.0 * 5.0 * 1000.0;
    rf_front_mm = rf_front_mVoltage * 5.0 / 4.88; // From Datasheet
    rf_front_in = (rf_front_mm * 0.0394);         // mm to inch conversion factor
    front_avg += rf_front_in;
  }
  front_avg /= 5;

  // Back is MB 1040 (in model)
  float rf_back_mVoltage = 0, rf_back_in = 0, back_avg = 0;
  for (int i = 0; i < 5; i++)
  {
    rf_back_mVoltage = analogRead(USPin2) / 1024.0 * 5.0 * 1000.0;
    rf_back_in = rf_back_mVoltage / 9.8; // From Datasheet
    back_avg += rf_back_in;
  }
  back_avg /= 5;
  rh.publishRangeFinders(rf_front_in, rf_back_in);

  // Publish Battery Levels
  publishBatLevels(bat_monitor.readBusVoltage(), bat_monitor.readCurrent());
  // THESE ARE NEEDED TO MAKE MOTOR SPIN ^^^^ -- theyre now in one function
  rf.range();
  override_was_active = true;
  sound_regulator++;
  rh.spin();
}

