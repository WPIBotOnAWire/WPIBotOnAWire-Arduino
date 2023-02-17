#include "teleop.h"

teleop::teleop(void){}

void teleop::init(void){
        pinMode(RADIO_OVERRIDE_PIN, INPUT);
}

bool teleop::check_radio_active() {
    bool active = pulseIn(RADIO_OVERRIDE_PIN, HIGH) > 1500;
    RH.publishRadioActive(active);
    return active;
}

int teleop::checkThrottle(){
    return pulseIn(RADIO_OVERRIDE_DETECT, HIGH);
}

void teleop::checkLightSwitch(void){
    int lights = pulseIn(RADIO_OVERRIDE_LIGHTS, HIGH);
    lights_control(lights);
}

void teleop::checkSoundKnob(void){
    int sounds = pulseIn(RADIO_OVERRIDE_SOUND, HIGH);
    sounds_control(sounds);

}

int checkDectectionKnob(void){
    return pulseIn(RADIO_OVERRIDE_DETECT, HIGH);
}

void teleop::lights_control(int light_switch){
    // light_switch is the pulseIn reading, checks if flipped up or down; down is ~1800-1900, up is ~1000-1100
    if (light_switch < 1500) digitalWrite(LED_PIN, HIGH);
    else digitalWrite(LED_PIN, LOW);
}

void teleop::sounds_control(int sound_switch){
    // sound_switch is a pulseIn reading, checks if knob is turned past midpoint ~1500 where under 1500 is off and over is on
    int frequency = 4000;
    if (sound_switch > 1500) {
        // sound_regulator will cause the tone to play once per second
        if (sound_regulator >= LOOP_CONSTANT) {
            RH.publishRadioActive(frequency);
            sound_regulator = 0;
        }
    }
}
