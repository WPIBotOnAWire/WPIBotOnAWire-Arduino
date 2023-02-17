#include "driveController.h"

ESC motor1(ESC1_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);
ESC motor2(ESC2_PIN, MOTOR_FULLBACK, MOTOR_FULLFORWARD, MOTOR_STOP);


void driveController::init(){
    arm();
    delay(500);
    stop();
}
void driveController::setSpeed(int throttle){
    // Sets the speed of the motors with a given input
    motor1.speed(throttle);
    motor2.speed(throttle);
    Serial.println("Driving at ");
    Serial.print(throttle);
    Serial.println("");
}

void driveController::drive_rpm(double target_speed){
    float pid_speed = EC.pid_effort_rpm(target_speed);
    // setSpeed(pid_speed);
    Serial.println("Driving at PID ");
    Serial.print(pid_speed);
    Serial.println("");
}

void driveController::drive_forward_inches(long inches){
    // if(dist_traveled >= inches){
    //     setSpeed(1500);
    // }else{
    //     setSpeed(1550);
    // }
}

void driveController::driveController(void){

}

void driveController::stop(){
    motor1.speed(MOTOR_STOP);
    motor2.speed(MOTOR_STOP);
}

void driveController::arm(){
    motor1.arm();
    motor2.arm();
}

