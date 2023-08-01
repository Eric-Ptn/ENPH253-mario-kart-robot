#pragma once
#include "config.h"

class TapeFollower {
    private:
        // raw analog readings
        int ir_readings[NUM_IR_SENSORS];

        // calibration values
        int ir_offsets[NUM_IR_SENSORS];
        double ir_scaling[NUM_IR_SENSORS]; // type double because it's a small ratio (less than 2)
        
        // tape following PID vals
        const double desired_center = NUM_IR_SENSORS / 2 + 0.5; // position indices in weighted average start from 1, so add 0.5 to get the center

        double proportional = 0;
        double derivative = 0;
        double integral = 0;
        double last_error = 0;
        double max_integral; // wind-up safety

        // tape following PID gains
        const double Kp = TAPE_FOLLOWING_KP;
        const double Kd = TAPE_FOLLOWING_KD;
        const double Ki = TAPE_FOLLOWING_KI;

    public:
        TapeFollower();

        // basic functions
        int processed_ir_reading(int sensor_index);
        int ir_reading_no_threshold(int sensor_index);

        // calibration
        void constant_offset_calibration();
        void scaling_offset_calibration(); 
        void quick_calibration(); // uses previously saved values in config
        void tape_calibration(); // recommended, scales sensitivity of each sensor to be about the same by calibrating on black and white

        // tape following
        /*
        Purpose: PID routine to follow tape. 
        */
        void follow_tape(double duty_cycle_offset = 0);

        // sensor events
        bool seeing_white();
        bool seeing_black();
        // bool test_bool();
};