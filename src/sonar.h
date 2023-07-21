#include <Arduino.h>
#include <Wire.h>
#include <global_values.h>
#include <oled_display.h>

/*
NOTE: SONAR MUST BE USED WITH VCC AT 5V. THE ECHO SIGNAL IS ALSO AT 5V SO IT MUST BE DIVIDED DOWN TO 3V BEFORE BEING SENT TO BLUEPILL
*/

/*
Purpose: Triggers the Tx of the sonar by supplying a PWM input with a period of about 60ms and a duty cycle of 1/60

Params:
 - Sonar: an int which represents which of the two sonar modules is being used.

Returns: Nothing.

*/
void trigger_sonar (int sonar_trigger_pin) {
    double duty_cycle_percent = 1/60.0 * 100.0;

    if (sonar_trigger_pin == BRIDGE_SONAR_TRIGGER) {
        pwm_start(BRIDGE_SONAR_PWM_NAME, SONAR_FREQUENCY, duty_cycle_percent, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }

    else if (sonar_trigger_pin == WALL_SONAR_TRIGGER) {
        pwm_start(WALL_SONAR_PWM_NAME, SONAR_FREQUENCY, duty_cycle_percent, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }
    
}


/*
Purpose: Uses the sonar to measure the distance between the sonar module and an obstacle. The width of the the pulse measured at the 
         echo pin represents the time taken for the signal to reach the Rx, using this makes it straighforward to find the distance.
Params:
 - Sonar: an int which represents which of the two sonar modules is being used.

 Returns: The distance between sonar module and obstruction.

 Note: Using longs since I don't think we need the precision of double here - Tanishq
 Note: aint no way - Eric
*/
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