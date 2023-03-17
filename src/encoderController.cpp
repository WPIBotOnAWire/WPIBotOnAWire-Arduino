#include "encoderController.h"

// encoder
Encoder encoder(ENCODER_PIN1, ENCODER_PIN2);


encoderController::encoderController(){

}

void encoderController::init(){

}

void encoderController::encoder_counts(void){
    encoderCounts = encoder.read();
    Serial.println("ENC: ");
}

float encoderController::rotations(){
    encoder_counts();
    Serial.print(encoderCounts);
    float rotations = encoderCounts/PPR;
    Serial.println("");
    Serial.println("Rot ");
    Serial.print(rotations);
    Serial.println("");
    return rotations;
}

float encoderController::rpm(float rotations){
    int time_end = millis();
    double rot_end = rotations;
    double rot_elapsed = rot_end-rot_start;
    int time_elapsed = time_end-time_start;
    float RPM = (rot_elapsed/time_elapsed)*1000*60;
    Serial.println("RPM: ");
    Serial.print(RPM);
    Serial.println("");
    time_start = time_end;
    rot_start = rotations;
    return RPM;
}

float encoderController::dist_traveled(float rotations){
    float dist_traveled = rotations * WHEEL_CIRCUMFRANCE;
    Serial.println("Dist traveled: ");
    Serial.print(dist_traveled);
    Serial.println("");
    return dist_traveled;
}

float encoderController::pid_effort_rpm(double target_speed){
    float diff_rpm = target_speed-rpm(rotations());
    float adj_speed = Kp*diff_rpm + Ki*diff_rpm;
    Serial.println("PID effort ");
    Serial.print(diff_rpm);
    Serial.println("");
    return adj_speed;
}

int encoderController::get_encoder_counts(void){
    encoder_counts();
    Serial.println("PID effort ");
    Serial.print(encoderCounts);
    Serial.println("");
    return encoderCounts;
};

// void loop(){
//     encoderController EC  = encoderController();
//     EC.get_encoder_counts();
// };
