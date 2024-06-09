#include "robot.h"
#include "ESmotor.h"
#include "led-ROS.h"

Robot robot;

// cm
const float APPROACHING_THRESHOLD = 200.0;
const float STOPPING_THRESHOLD = 50.0;
const float DETERRING_THRESHOLD = 100.0;

// m/s
const float PATROLLING_SPEED = 1.0; // m/s

void Robot::Arm(void) 
{
    robotState = ROBOT_DETERRING; //start slow... 
    robotDirection = DIR_HOLD; 
    esMotor.Arm();
    DEBUG_SERIAL.println("Arming");
}

void Robot::Disarm(void) 
{
    robotState = ROBOT_IDLE; 
    robotDirection = DIR_HOLD; 
    esMotor.EStop();
    DEBUG_SERIAL.println("Disarming");
}

void Robot::handleMaxBotixReading(float distanceCM, DIRECTION direction)  // needs to know what sensor
{
    nearestObjectCM[direction] = distanceCM;
    if(robotDirection == direction)
    {
        DEBUG_SERIAL.print("MB: ");
        DEBUG_SERIAL.print(distanceCM);

       if(robotState != ROBOT_IDLE)
        {
            float targetSpeed = PATROLLING_SPEED * (distanceCM - STOPPING_THRESHOLD) 
                                / (APPROACHING_THRESHOLD - STOPPING_THRESHOLD);  // CAREFUL IF NEGATIVE!!
            
            if(targetSpeed < 0) targetSpeed = 0;
            if(targetSpeed > PATROLLING_SPEED) targetSpeed = PATROLLING_SPEED;

            setTargetSpeed(targetSpeed);

            DEBUG_SERIAL.print("\tTarget:\t");
            DEBUG_SERIAL.print(targetSpeed);
        }

        DEBUG_SERIAL.print('\n');

        // We use if and not elseif so that the logic cascades
        if(robotState == ROBOT_PATROLLING)
        {
            if(distanceCM <= APPROACHING_THRESHOLD)
            {
                robotState = ROBOT_APPROACHING;
                DEBUG_SERIAL.println("Pat -> App");
            }
        }

        if(robotState == ROBOT_APPROACHING)
        {
            if(distanceCM <= DETERRING_THRESHOLD)
            {   
                setLED();
                robotState = ROBOT_DETERRING;
                DEBUG_SERIAL.println("App -> Det");
            }

            else if(distanceCM > APPROACHING_THRESHOLD)
            {
                robotState = ROBOT_PATROLLING;
                DEBUG_SERIAL.println("App -> Pat");
            }
        }

        if(robotState == ROBOT_DETERRING)
        {
            if(distanceCM <= STOPPING_THRESHOLD) //superfluous, but maybe we'll need it later?
            {
                robotState = ROBOT_DETERRING;
            }

            else if(distanceCM > DETERRING_THRESHOLD)
            {
                clearLED();
                robotState = ROBOT_APPROACHING;
                DEBUG_SERIAL.println("Det -> App");
            }
        }
    }
}

/**
 * Sets the direction for patrolling in auto. 
 * Ignore if called with current direction.
 */
void Robot::SetDirection(DIRECTION dir)
{
    if(robotDirection != dir) 
    {
        setTargetSpeed(0);
        robotDirection = dir;
        // DEBUG_SERIAL.println(dir);
    }
}

void Robot::setTargetSpeed(float speed)
{
    if(robotDirection == DIR_FWD) esMotor.SetTargetSpeedMetersPerSecond(speed);
    else if(robotDirection == DIR_REV) esMotor.SetTargetSpeedMetersPerSecond(-speed);
    else esMotor.SetTargetSpeedMetersPerSecond(0);
}

void Robot::handleEncoderUpdate(const float movementCM)
{
    nearestObjectCM[DIR_REV] += movementCM;
    nearestObjectCM[DIR_FWD] -= movementCM;
}
