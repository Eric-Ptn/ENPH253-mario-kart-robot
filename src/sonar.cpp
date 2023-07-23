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
}
