#pragma once
// NOTE: SONAR MUST BE USED WITH VCC AT 5V. THE ECHO SIGNAL IS ALSO AT 5V SO IT MUST BE DIVIDED DOWN TO 3V BEFORE BEING SENT TO BLUEPILL

namespace sonar {
    /*
    Purpose: Triggers the Tx of the sonar by supplying a PWM input with a period of about 60ms and a duty cycle of 1/60
    Params:
    - Sonar: an int which represents which of the two sonar modules is being used.
    */
    void trigger_sonar(int sonar_trigger_pin);

    /*
    Purpose: Uses the sonar to measure the distance between the sonar module and an obstacle. The width of the the pulse measured at the 
            echo pin represents the time taken for the signal to reach the Rx, using this makes it straighforward to find the distance.
    Params:
    - Sonar: an int which represents which of the two sonar modules is being used.
    
    Returns: The distance between sonar module and obstruction.

    Note: Using longs since I don't think we need the precision of double here - Tanishq
    Note: aint no way - Eric
    */
    long measure_distance(int sonar_echo_pin);
}