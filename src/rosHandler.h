#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "constants.h"
class rosHandler{
    public:
        rosHandler(void);
        void init(void);
        void spin(void);
        void publishRadioActive(bool active);
        void publishSpeakerVal(int frequency);
        void publishEncoderCounts(int encoderCounts);
        void publishRangeFinders(float rf_front, float rf_back);
        void publishFrontRF(float front_val);
        void publishBackRF(float rf_back);
        void logwarn(const char* msg);
        void loginfo(const char* msg);
        void rfSetup(void);
        bool connected(void);
        void cb_led(const std_msgs::Bool &msg);
        ros::NodeHandle nh;
        
    private:

        
        
        // messages
        std_msgs::Float32 enc_val, rf_front_val, rf_back_val;
        std_msgs::Bool man_override;
        std_msgs::Int32 speaker_val;

        
};