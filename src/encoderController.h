#include <Encoder.h>
#include <constants.h>
class encoderController{
public:

encoderController(void);
void init(void);
void encoder_counts(void);
float rotations(void);
float rpm(float rotations);
float dist_traveled(float rotations);
void pid_effort_rpm(double target_speed);


private:
int encoderCounts = 0;
int prevEncoderCounts = 0;
int time_start;
double rot_start;
const float Kp = .3; 
const float Ki = 0.1;


};