#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ESC.h>
#include <Encoder.h>

#include "constants.h"

#define USE_USBCON

//For MaxSonar
#define USPin1 A0  //front
#define USPin2 A3    //back

ESC esc1(ESC1_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);
ESC esc2(ESC2_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);

Encoder encoder(ENCODER_PIN1, ENCODER_PIN2);

ros::NodeHandle nh;
std_msgs::Float32 enc_val, rf_front_val, rf_back_val;
ros::Publisher pub_enc("/encoder", &enc_val);
ros::Publisher pub_rf_front("/rangefinder/front", &rf_front_val);
ros::Publisher pub_rf_back("/rangefinder/back", &rf_back_val);

void cb_led(const std_msgs::Bool &msg) {
    int state = msg.data ? HIGH : LOW;

    digitalWrite(LED_PIN, state);
};

void cb_speaker(const std_msgs::Bool &msg) {

}

void cb_motor(const std_msgs::Float32 &msg) {
    int speed = map(msg.data, -1, 1, MOTOR_FULLBACK, MOTOR_FULLFORWARD);

    esc1.speed(speed);
    esc2.speed(speed);
}

ros::Subscriber<std_msgs::Bool> led_sub("/deterrents/led", &cb_led);
ros::Subscriber<std_msgs::Bool> speaker_sub("/deterrents/speaker", &cb_speaker);
ros::Subscriber<std_msgs::Float32> motor_sub("/motor_speed", &cb_motor);

void setup() {
    pinMode(LED_PIN, OUTPUT);

    esc1.arm();
    esc2.arm();

    delay(500);

    esc1.speed(MOTOR_STOP);
    esc2.speed(MOTOR_STOP);

    nh.initNode();
    nh.advertise(pub_enc);
    nh.advertise(pub_rf_back);
    nh.advertise(pub_rf_front);
}

void loop() {
    // Publish Encoder
    enc_val.data = encoder.read();
    pub_enc.publish(&enc_val);

    // Publish Rangefinders
    rf_front_val.data = (analogRead(USPin1) / 1024.0) * 512 * 2.54;
    pub_rf_front.publish(&rf_front_val);
    rf_back_val.data = (analogRead(USPin2) / 1024.0) * 512 * 2.54;
    pub_rf_back.publish(&rf_back_val);

    nh.spinOnce();
}
