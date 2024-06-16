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
    DEBUG_SERIAL.println("Arming");
    robotState = ROBOT_PATROLLING; 
    robotDirection = DIR_HOLD; 
    esMotor.Arm();
}

void Robot::Disarm(void) 
{
    robotState = ROBOT_IDLE; 
    robotDirection = DIR_HOLD; 
    esMotor.EStop();
    DEBUG_SERIAL.println("Disarming");
}

void Robot::Override(void) 
{
    robotState = RADIO_OVERRIDE; 
    robotDirection = DIR_HOLD; // ignored in override mode 
    esMotor.Arm();
    DEBUG_SERIAL.println("Overriding");
}

void Robot::FullStop(void)
{
    esMotor.FullStop();
}

void Robot::SwitchDirections(void)
{
    if(robotState != RADIO_OVERRIDE)
    {
        DEBUG_SERIAL.println("Switching directions");
        if(robotDirection == DIR_FWD) SetDirection(DIR_REV);
        else if(robotDirection == DIR_REV) SetDirection(DIR_FWD);
    }
}

void Robot::handleMaxBotixReading(float distanceCM, DIRECTION direction)  // needs to know what sensor
{
    nearestObjectCM[direction] = distanceCM;
    if(robotDirection == direction)
    {
#ifdef __DEBUG_MB__
        DEBUG_SERIAL.print("MB: ");
        DEBUG_SERIAL.print(distanceCM);
#endif
       if(robotState != ROBOT_IDLE && robotState != RADIO_OVERRIDE)
        {
            float targetSpeed = PATROLLING_SPEED * (distanceCM - STOPPING_THRESHOLD) 
                                / (APPROACHING_THRESHOLD - STOPPING_THRESHOLD);  // CAREFUL IF NEGATIVE!!
            
            if(targetSpeed < 0) targetSpeed = 0;
            if(targetSpeed > PATROLLING_SPEED) targetSpeed = PATROLLING_SPEED;

            setTargetSpeed(targetSpeed);

#ifdef __DEBUG_MB__
            DEBUG_SERIAL.print("\tTarget:\t");
            DEBUG_SERIAL.print(targetSpeed);
#endif
        }

#ifdef __DEBUG_MB__
        DEBUG_SERIAL.print('\n');
#endif
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
                deterrenceTimer.Start(5000);
            }

            else if(distanceCM > APPROACHING_THRESHOLD)
            {
                robotState = ROBOT_PATROLLING;
                DEBUG_SERIAL.println("App -> Pat");
            }
        }

        if(robotState == ROBOT_DETERRING)
        {
            if(distanceCM <= STOPPING_THRESHOLD) 
            {
                FullStop();
                robotState = ROBOT_STOPPED;
                DEBUG_SERIAL.println("Det -> Stp");
            }

            else if(distanceCM > DETERRING_THRESHOLD)
            {
                clearLED();
                robotState = ROBOT_APPROACHING;
                DEBUG_SERIAL.println("Det -> App");
            }
        }

        if(robotState == ROBOT_STOPPED)
        {
        if(distanceCM > STOPPING_THRESHOLD) 
            {
                robotState = ROBOT_DETERRING;
                DEBUG_SERIAL.println("Stp -> Det");
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
        FullStop();
        robotDirection = dir;
        DEBUG_SERIAL.print("Dir -> ");
        DEBUG_SERIAL.println(dir);
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
