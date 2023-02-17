#include "constants.h"
#include "rosHandler.h"

class teleop{
    public:
        void init(void);
        teleop(void);
        bool check_radio_active(void);
        int checkThrottle(void);
        void checkLightSwitch(void);
        void checkSoundKnob(void);
        int checkDectectionKnob(void);
        int sound_regulator = 0;
    private:
        void lights_control(int light_switch);
        void sounds_control(int sound_switch);
  
};