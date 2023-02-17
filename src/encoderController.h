#include <Encoder.h>
#include <constants.h>
class encoderController{
public:
    encoderController(void);
    void init(void);
    float rotations(void);
    float rpm(float rotations);
    float dist_traveled(float rotations);
    float pid_effort_rpm(double target_speed);
    int get_encoder_counts(void);


private:
    int encoderCounts = 0;
    int prevEncoderCounts = 0;
    int time_start;
    double rot_start;
    const float Kp = .3; 
    const float Ki = 0.1;
    void encoder_counts(void);
};