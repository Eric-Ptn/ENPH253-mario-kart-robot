#pragma once
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <deque>
#include "config.h"

class IMU{
    private:
        // IMU object
        Adafruit_MPU6050 imu;

        // raw readings
        double gyro_readings[3];
        double accel_readings[3];
        std::deque<double> z_accel_history;

        // calibration values
        double gyro_offsets[3];
        double accel_offsets[3];
        double gyro_z_drift = 0;
        double accel_x_drift = 0;
        double velocity_drift = 0;
        double angle_drift = 0; // 0.01 rad per 2 min

        // gyro calculating angle
        double angle = 0;
        double angle_y = 0;
        double velocity = 0;
        double last_imu_time = 0; // time of last imu reading

        // gyro PID vals
        double proportional = 0;
        double derivative = 0;
        double integral = 0;
        double last_error = 0;
        double max_integral = GYRO_MAX_INTEGRAL; // windup safety

        // gyro PID gains
        const double Kp = GYRO_KP;
        const double Kd = GYRO_KD;
        const double Ki = GYRO_KI;

    public:
        // BASIC FUNCTIONS *****************************************************************************************************************
        void begin_imu();
        void read_imu();
        // void i2c_reboot();

        /*
        Purpose: Calculates the angle that the gyro is oriented at. This angle is in the range -pi to pi.
                Calculates the speed that the gyro is moving forward at.
        Alters: angle (angle of the gyro, calibrated to 0 at beginning), velocity (speed of the gyro, calibrated to 0 at beginning)
        NOTE*: This function MUST be called in every loop iteration of the main program to keep integration up to date.
        */
        void calculate_quantities();
        void calculate_quantities_print();

        // CALIBRATION **********************************************************************************************************************

        /*
        Purpose: Sets IMU offsets such that the gyro and accelerometer readings are 0 when the IMU is stationary.
        Alters: IMU offsets
        NOTE*: THE GYRO MUST BE STATIONARY WHEN CALIBRATING.
        */
        void reading_calibrate();

        /*
        Purpose: Measures the drift of the z-angle of the gyro and the x-acceleration over a calibration period. Sets z_drift and x-acceleration according to a linear fit. 
        Alters: gyro_z_drift, accel_x_drift (both should only be altered by this function)
        */
        void drift_calibrate();
        void quick_calibration(); // uses previously saved values in config
        void velocity_linear_correction(); // corrects velocity for drift using linear least squares fit
        void angle_linear_correction();

        void reset_quantities();
        void new_lap();
        void calibrate_at_ramp();
        void set_pitch_to_zero();


        // SENSOR EVENTS *******************************************************************************************************************
        
        // detects accelerometer spike to judge whether robot is falling or not
        bool bumpy_terrain(double threshold);
        bool rubble_falling_edge();
        bool rubble_rising_edge();
        bool correct_orientation(double target_angle);
        bool negative_angle();
        bool correct_y_orientation_at_ramp();

        // HELPERS *************************************************************************************************************************
        double circular_correction(double angle);

        // OBJECTS *************************************************************************************************************************
        
        // object to control and track a single gyro-controlled motion
        class GyroMovement {
            private:
                bool completed = false;
                IMU* imu;
                double last_motor_time = 0;

            public:
                GyroMovement(IMU &parent_imu);

                /*
                Purpose: Turns to a specified angle.
                Params:
                - absolute_angle: specifies the angle that the vehicle is to be turned to.
                - servo_steering_angle: absolute value of angle that the servo will steer the vehicle at (specifies the turning radius)
                */
                void gyro_turn_absolute(double absolute_angle, double servo_steering_angle, double duty_cycle_offset = 0);

                /*
                Purpose: Turns vehicle BY a specified angle (the nuance is in the wording).
                Params:
                - turning_angle: specifies the angle that the vehicle is to be turned BY.
                - servo_steering_angle: absolute value of angle that the servo will steer the vehicle at (specifies the turning radius)
                */
                void gyro_turn_relative(double turn_angle, double servo_steering_angle, double duty_cycle_offset = 0);

                /*
                Purpose: Uses PID control and the gyro to drive along a given angle. 
                Params: 
                - target_angle: the angle to drive the vehicle at.
                - stop_condition: a function that returns true when the vehicle should stop driving.
                */
                void gyro_drive_straight_angle(double target_angle, std::function<bool()> stop_condition, double duty_cycle_offset = 0);

                bool complete();
        };
        
};