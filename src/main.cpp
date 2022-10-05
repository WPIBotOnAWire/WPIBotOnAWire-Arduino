#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ESC.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include <sensor_msgs/BatteryState.h>

#include "constants.h"

#define USE_USBCON

Adafruit_INA260 bat_monitor = Adafruit_INA260();

ESC esc1(ESC1_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);
ESC esc2(ESC2_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);

Encoder encoder(ENCODER_PIN1, ENCODER_PIN2);

ros::NodeHandle nh;
std_msgs::Float32 enc_val, rf_front_val, rf_back_val;
std_msgs::Bool man_override;
std_msgs::Int32 speaker_val;
sensor_msgs::BatteryState bat_msg;
ros::Publisher pub_enc("/encoder", &enc_val);
ros::Publisher pub_rf_front("/rangefinder/front", &rf_front_val);
ros::Publisher pub_rf_back("/rangefinder/back", &rf_back_val);
ros::Publisher pub_bat_level("/battery", &bat_msg);
ros::Publisher pub_man_override("/manual_override", &man_override);
ros::Publisher pub_speakers("/play_sound", &speaker_val);

int sound_regulator = 0;

bool override_was_active = false;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void cb_led(const std_msgs::Bool &msg) {
    int state = msg.data ? HIGH : LOW;
    digitalWrite(LED_PIN, state);
}

void cb_motor(const std_msgs::Float32 &msg) {
    int speed = mapfloat(msg.data, -1.0, 1.0, MOTOR_FULLBACK, MOTOR_FULLFORWARD);
    esc1.speed(speed);
    esc2.speed(speed);
}

ros::Subscriber<std_msgs::Bool> led_sub("/deterrents/led", &cb_led);
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
    nh.advertise(pub_bat_level);
    nh.advertise(pub_man_override);
    nh.advertise(pub_speakers);

    nh.subscribe(led_sub);
    nh.subscribe(motor_sub);

    bat_monitor.begin();

    pinMode(RADIO_OVERRIDE_PIN, INPUT);
    Serial.begin(9600); // when running robot.launch, comment this out
}

bool check_radio_active() {
    bool active = pulseIn(RADIO_OVERRIDE_PIN, HIGH) > 1500;
    man_override.data = active;
    pub_man_override.publish(&man_override);
    return active;
}

void lights_control(int light_switch){
    // light_switch is the pulseIn reading, checks if flipped up or down; down is ~1800-1900, up is ~1000-1100
    if (light_switch < 1500) digitalWrite(LED_PIN, HIGH);
    else digitalWrite(LED_PIN, LOW);
}

void sounds_control(int sound_switch){
    // sound_switch is a pulseIn reading, checks if knob is turned past midpoint ~1500 where under 1500 is off and over is on
    if (sound_switch > 1500) {
        speaker_val.data = 4000;
        // sound_regulator will cause the tone to play once per second
        if (sound_regulator >= LOOP_CONSTANT) {
            pub_speakers.publish(&speaker_val);
            sound_regulator = 0;
        }
    }
}

int keepDistance(float rf_front_in, float rf_back_in){
    // Prevents robot from colliding with objects
    // Returns speed for motor
    
    // Gather data
    int throttle = pulseIn(RADIO_OVERRIDE_THROTTLE, HIGH);
    
    // Set up maximum speeds for either direction
    int front_max = 1425, back_max = 1580;
    int f_error, b_error; 

    // Slows down robot as it moves closer to an object (starts slowing down at a safe distance, can't go further than stop distance)
    // Contstrain speed for objects in front, between 
    if(rf_front_in < APPROACH_DISTANCE){
        // Deterimine error
        f_error = APPROACH_DISTANCE - rf_front_in;

        front_max += f_error * ((MOTOR_STOP - front_max) / (APPROACH_DISTANCE-STOP_DISTANCE));

        if(rf_front_in < STOP_DISTANCE){
            front_max = 1500;
        }
        
    }
    // Constrain speed for objects behind
    if(rf_back_in < APPROACH_DISTANCE){
        // Deterimine error
        b_error = APPROACH_DISTANCE - rf_back_in;

        back_max -= b_error * (abs(MOTOR_STOP - back_max) / (APPROACH_DISTANCE-STOP_DISTANCE));

        if(rf_back_in < STOP_DISTANCE){
            back_max = 1500;
        }
        
    }

    // Set speed based on constraints
    if (throttle >= 1450 && throttle <= 1550) throttle = 1500;
    if (throttle < front_max) throttle = front_max;
    if (throttle > back_max) throttle = back_max;

    return throttle;
}

void setSpeed(int throttle){
    // Sets the speed of the motors with a given input
    esc1.speed(throttle);
    esc2.speed(throttle);
}

void detectMode(int detect_pin, int front_avg, int back_avg){
    if (detect_pin > 1500){
        // check for object
        if((back_avg < STOP_DISTANCE) || (front_avg < STOP_DISTANCE)){
            // need to make the robot slow to a stop and not hit the bird
            
            // turn on lights
            lights_control(1000);

            // turn on sound
            sounds_control(1900);
        } else digitalWrite(LED_PIN, LOW);
    }
}

int time_start = 0;
double rot_start = 0;
void encoder_counts(){
    int PPR = 1024; //PPR = pulses per revolution
    int enc = encoder.read();
    double rotations = enc/PPR;
    int time_end = millis();
    double rot_end = rotations;
    double rot_elapsed = rot_end-rot_start;
    int time_elapsed = time_end-time_start;
    double RPM = (rot_elapsed/time_elapsed)*1000*60;
    Serial.println("RPM: ");
    Serial.print(RPM);
    Serial.println("");
    time_start = time_end;
    rot_start = rotations;
}

void loop() {
    encoder_counts();
    int throttle = pulseIn(PIN_A6, HIGH);
    //Serial.print("THROTTLE ");
    //Serial.println(throttle);
    // Publish Encoder
    enc_val.data = encoder.read();
    pub_enc.publish(&enc_val);

    // Publish Rangefinders
    //Front is MB 1043 (mm model)
    float rf_front_mVoltage = 0, rf_front_mm = 0, rf_front_in = 0, front_avg = 0;
    for (int j = 0; j < 5; j++){
        rf_front_mVoltage = analogRead(USPin1)/1024.0*5.0*1000.0;
        rf_front_mm = rf_front_mVoltage * 5.0 / 4.88; //From Datasheet
        rf_front_in = (rf_front_mm * 0.0394); //mm to inch conversion factor
        front_avg += rf_front_in;
    }
    front_avg /= 5;
    rf_front_val.data = (rf_front_in);
    pub_rf_front.publish(&rf_front_val);

    //Back is MB 1040 (in model)
    float rf_back_mVoltage = 0, rf_back_in = 0, back_avg = 0;
    for (int i = 0; i < 5; i++){
        rf_back_mVoltage = analogRead(USPin2)/1024.0*5.0*1000.0;
        rf_back_in = rf_back_mVoltage / 9.8; //From Datasheet
        back_avg += rf_back_in;
    }
    back_avg /= 5;
    rf_back_val.data = (rf_back_in);
    pub_rf_back.publish(&rf_back_val);

    // Publish Battery Levels
    bat_msg.voltage = bat_monitor.readBusVoltage();
    bat_msg.current = bat_monitor.readCurrent();

    pub_bat_level.publish(&bat_msg);
    check_radio_active();
    if (check_radio_active()) {
        // Read radio values and use them
        int lights = pulseIn(RADIO_OVERRIDE_LIGHTS, HIGH);
        int sounds = pulseIn(RADIO_OVERRIDE_SOUND, HIGH);
        int detect_pin = pulseIn(RADIO_OVERRIDE_DETECT, HIGH);

        // check if light switch is up through values from pulseIn + turn lights on/off
        lights_control(lights);

        // makes a beep go off if the knob is turned to the right
        sounds_control(sounds);

        // Detection Mode
        detectMode(detect_pin, front_avg, back_avg);

        // Alter speed based on distance detected from an object
        int ctrl_speed = keepDistance(front_avg, back_avg);

        // Set speed
        //Serial.print("DRIVING AT ");
        //Serial.print(ctrl_speed);
        //Serial.println("");
        setSpeed(ctrl_speed);

        override_was_active = true;
    } else {
        if(override_was_active) {
            //Serial.println("Stopped");
            esc1.speed(MOTOR_STOP);
            esc2.speed(MOTOR_STOP);

            override_was_active = false;
        }

    }
    sound_regulator++;
    nh.spinOnce();
}