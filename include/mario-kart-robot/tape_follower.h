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
        static const double Kp;
        static const double Kd;
        static const double Ki;

    public:
        TapeFollower();

        // basic functions
        int processed_ir_reading(int sensor_index);

        // calibration
        void constant_offset_calibration();
        void scaling_offset_calibration();

        // tape following
        /*
        Purpose: PID routine to follow tape. 
        */
        void follow_tape();

        // sensor events
        bool seeing_white();
        bool seeing_black();
};