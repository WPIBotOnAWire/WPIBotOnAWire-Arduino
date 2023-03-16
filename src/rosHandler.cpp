#include "rosHandler.h"

        ros::NodeHandle nh;
        
        // messages
        std_msgs::Float32 enc_val, rf_front_val, rf_back_val;
        std_msgs::Bool man_override;
        std_msgs::Int32 speaker_val;
        sensor_msgs::BatteryState bat_msg;




        //subscribers
        ros::Subscriber<std_msgs::Bool> led_sub("/deterrents/led", &cb_led);
        ros::Subscriber<std_msgs::Float32> motor_sub("/motor_speed", &cb_motor);
        // publishers
        ros::Publisher pub_enc("/encoder", &enc_val);
        ros::Publisher pub_rf_front("/rangefinder/front", &rf_front_val);
        ros::Publisher pub_rf_back("/rangefinder/back", &rf_back_val);
        ros::Publisher pub_bat_level("/battery", &bat_msg);
        ros::Publisher pub_man_override("/manual_override", &man_override);
        ros::Publisher pub_speakers("/play_sound", &speaker_val);

        rosHandler::rosHandler(void){

        }

    void cb_led(const std_msgs::Bool &msg) {
        int state = msg.data ? HIGH : LOW;
        digitalWrite(LED_PIN, state);
    }

    void cb_motor(const std_msgs::Float32 &msg) {
    //int speed = mapfloat(msg.data, -1.0, 1.0, MOTOR_FULLBACK, MOTOR_FULLFORWARD);
    int speed = msg.data;
    // esc1.speed(speed);
    // esc2.speed(speed);
    }
    
    void rosHandler::init(void){
        nh.initNode();
        nh.advertise(pub_enc);
        nh.advertise(pub_rf_back);
        nh.advertise(pub_rf_front);
        nh.advertise(pub_bat_level);
        nh.advertise(pub_man_override);
        nh.advertise(pub_speakers);

        nh.subscribe(led_sub);
        nh.subscribe(motor_sub);
    }

    void rosHandler::spin(void){
        nh.spinOnce();
    }


    void rosHandler::publishRadioActive(bool active){
        man_override.data = active;
        pub_man_override.publish(&man_override);
    }

    void rosHandler::publishSpeakerVal(int frequency){
        speaker_val.data = frequency;
        pub_speakers.publish(&speaker_val);
    }

    void rosHandler::publishEncoderCounts(int encoderCounts){
        enc_val.data = encoderCounts;
        pub_enc.publish(&enc_val);
    }

    void rosHandler::publishRangeFinders(float rf_front, float rf_back){
        rf_front_val.data = (rf_front);
        pub_rf_front.publish(&rf_front_val);

        rf_back_val.data = (rf_back);
        pub_rf_front.publish(&rf_back_val);
    }

    void rosHandler::publishBatLevels(float voltage,float current){
                bat_msg.voltage = voltage;
                bat_msg.current = current;

                pub_bat_level.publish(&bat_msg);
        }
