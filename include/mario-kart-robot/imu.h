#pragma once
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "config.h"

class IMU{
    private:
        // IMU object
        Adafruit_MPU6050 imu;

        // raw readings
        double gyro_readings[3];
        double accel_readings[3];

        // calibration values
        double gyro_offsets[3];
        double accel_offsets[3];
        double gyro_z_drift = 0;
        double accel_x_drift = 0;

        // gyro calculating angle
        double angle = 0;
        double angle_time = 0;
        double velocity = 0;
        double velocity_time = 0;

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
        void read_imu_screen();

        /*
        Purpose: Calculates the angle that the gyro is oriented at. This angle is in the range -pi to pi.
        Alters: angle (angle of the gyro, calibrated to 0 at beginning)
        */
        double calculate_z_angle();
        double calculate_velocity();

        // CALIBRATION **********************************************************************************************************************

        /*
        Purpose: Sets IMU offsets such that the gyro and accelerometer readings are 0 when the IMU is stationary.
        Alters: IMU offsets
        NOTE*: THE GYRO MUST BE STATIONARY WHEN CALIBRATING.
        */
        void reading_calibrate();

        /*
        Purpose: Measures the drift of the z-angle of the gyro over a calibration period. Sets z_drift according to a linear fit.
        Alters: gyro_z_drift (should only be altered by this function)
        */
        void drift_calibrate();

        void reset_angle();
        void reset_speed();

        // MOVEMENT ************************************************************************************************************************

        

        // SENSOR EVENTS *******************************************************************************************************************
        
        // detects accelerometer spike to judge whether robot is falling or not
        bool robot_falling();

        // HELPERS *************************************************************************************************************************
        double circular_correction(double angle);

        // OBJECTS *************************************************************************************************************************
        
        // gyro turn object
        class GyroMovement {
            private:
                bool completed = false;
                IMU* imu;

            public:
                GyroMovement(IMU &parent_imu);

                /*
                Purpose: Turns to a specified angle.
                Params:
                - absolute_angle: specifies the angle that the vehicle is to be turned to.
                - servo_steering_angle: absolute value of angle that the servo will steer the vehicle at (specifies the turning radius)
                */
                void gyro_turn_absolute(double absolute_angle, double servo_steering_angle);

                /*
                Purpose: Turns vehicle BY a specified angle (the nuance is in the wording).
                Params:
                - turning_angle: specifies the angle that the vehicle is to be turned BY.
                - servo_steering_angle: absolute value of angle that the servo will steer the vehicle at (specifies the turning radius)
                */
                void gyro_turn_relative(double turn_angle, double servo_steering_angle);

                /*
                Purpose: Uses PID control and the gyro to drive along a given angle. 
                Params: 
                - target_angle: the angle to drive the vehicle at.
                - stop_condition: a function that returns true when the vehicle should stop driving.
                */
                void gyro_drive_straight_angle(double target_angle, std::function<bool()> stop_condition);

                bool complete();
        };
        
};