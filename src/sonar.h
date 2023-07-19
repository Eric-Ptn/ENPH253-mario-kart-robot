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
void trigger_sonar (int sonar) {
    double duty_cycle = 1/60.0 * 100.0;

    if (sonar == 1) {
        pwm_start(SONAR_PIN_1_PWM_NAME, frequency, duty_cycle, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }

    else if (sonar == 2) {
        pwm_start(SONAR_PIN_2_PWM_NAME, frequency, duty_cycle, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }
    
}


/*
Purpose: Uses the sonar to measure the distance between the sonar module and an obstacle. The width of the the pulse measured at the 
         echo pin represents the time taken for the signal to reach the Rx, using this makes it straighforward to find the distance.
Params:
 - Sonar: an int which represents which of the two sonar modules is being used.

 Returns: The distance between sonar module and obstruction.

 Note: Using longs since I don't think we need the precision of double here - Tanishq
*/
long measure_distance (int sonar) {
    long duration = 10000;

    if (sonar == 1) {
        duration = pulseIn(SONAR_ECHO_PIN_1, HIGH);
    }

    if (sonar == 2) {
        duration = pulseIn(SONAR_ECHO_PIN_2, HIGH);
    }
    

    long distance = duration * 0.0343 / 2.0;

    return distance;
}