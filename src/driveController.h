#include "constants.h"

class driveController{
    void driveController(void);
    void setSpeed(int throttle);
    void drive_rpm(double target_speed);
    void drive_inches(long inches);
};