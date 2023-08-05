#include <Arduino.h>
#include <Wire.h>
#include <mario-kart-robot\config.h>

namespace sonar {
    void trigger_sonar (int sonar_trigger_pin) {
        double duty_cycle_percent = 1/60.0 * 100.0;

        if (sonar_trigger_pin == BRIDGE_SONAR_TRIGGER) {
            pwm_start(BRIDGE_SONAR_PWM_NAME, SONAR_FREQUENCY, duty_cycle_percent, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
        }

        else if (sonar_trigger_pin == WALL_SONAR_TRIGGER) {
            pwm_start(WALL_SONAR_PWM_NAME, SONAR_FREQUENCY, duty_cycle_percent, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
        }
    
    }


    long measure_distance (int sonar_echo_pin) {
        long duration = 10000;

        if (sonar_echo_pin == BRIDGE_SONAR_ECHO) {
            duration = pulseIn(BRIDGE_SONAR_ECHO, HIGH);
        }

        if (sonar_echo_pin == WALL_SONAR_ECHO) {
            duration = pulseIn(WALL_SONAR_ECHO, HIGH);
        }
        

        long distance = duration * 0.0343 / 2.0; // distance is time taken for pulse to return * speed of sound / 2 (since it travels to and from the object)

        return distance;
    }

    void stop_sonar(int sonar_trigger_pin){
        if (sonar_trigger_pin == BRIDGE_SONAR_TRIGGER) {
            pwm_start(BRIDGE_SONAR_PWM_NAME, SONAR_FREQUENCY, 0, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
        }

        else if (sonar_trigger_pin == WALL_SONAR_TRIGGER) {
            pwm_start(WALL_SONAR_PWM_NAME, SONAR_FREQUENCY, 0, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
        }
    }

    bool seeing_bridge(){
        return measure_distance(BRIDGE_SONAR_ECHO) < BRIDGE_DISTANCE;
    }

    bool seeing_bridge_falling_edge(){
        static bool last_seen = false;

        if (seeing_bridge() && !last_seen) {
            last_seen = true;
            return false;
        }
        if (!seeing_bridge() && last_seen) {
            last_seen = false;
            return true;
        }
        return false;
    }

    bool seeing_wall(){
        return measure_distance(WALL_SONAR_ECHO) < WALL_DISTANCE;
    }

    bool test_bool(){
        // if (millis() > 50000) {
        //     return true;
        // }
        return false;
    }
}
