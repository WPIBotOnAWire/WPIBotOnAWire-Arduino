#include <Arduino.h>
#include <rosHandler.h>
#include <ESC.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include "encoderController.h"
#include "tof.h"
#include "teleop.h"

#define USE_USBCON

// battery Monitor

Adafruit_INA260 bat_monitor = Adafruit_INA260();
DriveController drive = driveController();
encoderController EC = encoderController();
teleop tele = teleop();
rosHandler RH = rosHandler();
int encoder_counts=0;
bool override_was_active = false;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    drive.init();
    RH.init();
    EC.init();
    bat_monitor.begin();
    tele.init();
    Serial.begin(9600); // when running robot.launch, comment this out
}

// commented out until we restructure it to match all the stuff I moved

// void detectMode(int detect_pin, int front_avg, int back_avg){
//     if (detect_pin > 1500){
//         // check for object
//         if((back_avg < STOP_DISTANCE) || (front_avg < STOP_DISTANCE)){
//             // need to make the robot slow to a stop and not hit the bird
            
//             // turn on lights
//             lights_control(1000);

//             // turn on sound
//             sounds_control(1900);
//         } else digitalWrite(LED_PIN, LOW);
//     }
// }

void loop() {
    
    // restructure encoder count reads

    encoder_counts = EC.get_encoder_counts();
    drive.drive_rpm(0);
    int throttle = tele.checkThrottle();
    //Serial.print("THROTTLE ");
    //Serial.println(throttle);
    
    // Publish Encoder
    RH.publishEncoderCounts(encoder_counts);

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


    //Back is MB 1040 (in model)
    float rf_back_mVoltage = 0, rf_back_in = 0, back_avg = 0;
    for (int i = 0; i < 5; i++){
        rf_back_mVoltage = analogRead(USPin2)/1024.0*5.0*1000.0;
        rf_back_in = rf_back_mVoltage / 9.8; //From Datasheet
        back_avg += rf_back_in;
    }
    back_avg /= 5;

    RH.publishRangeFinders(rf_front_in,rf_back_in);

    // Publish Battery Levels
    float voltage = bat_monitor.readBusVoltage();
    float current = bat_monitor.readCurrent();

    RH.publishBatLevels(voltage, current);

    if (tele.check_radio_active()) {

        // check if light switch is up through values from pulseIn + turn lights on/off
        teleop.checkLightSwitch();
        
        // makes a beep go off if the knob is turned to the right
        teleop.checkSoundKnob();
        
        // Detection Mode
        // detectMode(teleop.checkDetectionKnob, front_avg, back_avg);

        // Alter speed based on distance detected from an object
        int ctrl_speed = drive.keepDistance(front_avg, back_avg);
        drive.setSpeed(ctrl_speed);

        override_was_active = true;
    } else {
        if(override_was_active) {
            //Serial.println("Stopped");
            drive.stop();
            override_was_active = false;
        }

    }
    teleop.sound_regulator++;
    RH.spin();
} 