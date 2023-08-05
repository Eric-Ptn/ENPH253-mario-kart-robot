#pragma once
#include <math.h>

// pins
#define SERVO_PIN PA6
#define SERVO_PIN_PWM_NAME PA_6

#define LEFT_MOTOR_PIN PB6
#define LEFT_MOTOR_PIN_PWM_NAME PB_6
// #define LEFT_REVERSE_MOTOR_PIN PB7
// #define LEFT_REVERSE_MOTOR_PIN_PWM_NAME PB_7
#define LEFT_REVERSE_MOTOR_PIN PB8
#define LEFT_REVERSE_MOTOR_PIN_PWM_NAME PB_8

// #define RIGHT_MOTOR_PIN PB8
// #define RIGHT_MOTOR_PIN_PWM_NAME PB_8
#define RIGHT_MOTOR_PIN PB7
#define RIGHT_MOTOR_PIN_PWM_NAME PB_7
#define RIGHT_REVERSE_MOTOR_PIN PB9
#define RIGHT_REVERSE_MOTOR_PIN_PWM_NAME PB_9

#define BRIDGE_SONAR_TRIGGER PA8
#define BRIDGE_SONAR_PWM_NAME PA_8
#define BRIDGE_SONAR_ECHO PA11

#define WALL_SONAR_TRIGGER PA9
#define WALL_SONAR_PWM_NAME PA_9  
#define WALL_SONAR_ECHO PA12

#define START_BUTTON PA10

// #define CALIBRATION_PIN PB3
// #define RUNNING_PIN PB4

// gyro and OLED connect to 2nd I2C pins, PB10 and PB11


// IR sensor pins
const byte IR_PINS[] = {PA1, PA0, PA4, PA3};
const int NUM_IR_SENSORS = sizeof(IR_PINS) / sizeof(IR_PINS[0]);

// gyro PID values
#define GYRO_KP 1
#define GYRO_KI 0
#define GYRO_KD 0
#define GYRO_MAX_INTEGRAL 1

// tape following PID values
#define TAPE_FOLLOWING_KP 0.2
#define TAPE_FOLLOWING_KI 0
#define TAPE_FOLLOWING_KD 0
#define TAPE_FOLLOWING_MAX_INTEGRAL 1

// gyro calibration values
#define IMU_FAST_CALIBRATION_RUNS 1000
#define IMU_SLOW_CALIBRATION_SECONDS 8
#define IMU_VELOCITY_FIT_RUNS 50

// ir calibration
#define IR_CALIBRATION_RUNS 5000
#define ERROR_MEMORY_THRESHOLD -500 // if the sum of analog readings is more than this threshold, then the previous error is used - effectively separates white and black
#define WHITE_VALUE 0
#define BLACK_VALUE -700
#define WHITE_THRESHOLD -400

// ir and gyro quick calibration - i could use EEPROM but that's too much work
const double SAVED_IR_SCALING[] = {1.0, 1.0, 1.0, 1.0};
const int SAVED_IR_OFFSETS[] = {0, 0, 0, 0};
const double SAVED_GYRO_OFFSETS[] = {0, 0, 0};
const double SAVED_ACCEL_OFFSETS[] = {0, 0, 0};
const int SAVED_Z_ANGLE_DRIFT = 0;
const int SAVED_X_ACCEL_DRIFT = 0;
const int SAVED_VELOCITY_DRIFT = 0;

// PWM frequencies (https://components101.com/motors/mg996r-servo-motor-datasheet)
#define SERVO_FREQUENCY_HZ 300 // THIS SHOULD NOT CHANGE
#define MOTOR_FREQUENCY_HZ 100 // this can change within reason - should not start hearing high-pitched whine

// motor speeds
#define DEFAULT_MOTOR_DUTY_CYCLE 35 // duty cycle for driving straight on smooth terrain
#define MAX_DUTY_CYCLE_BOOST_OUTER 4  // max duty cycle boost for sharpest turn
#define MAX_DUTY_CYCLE_BOOST_INNER -25

#define REDUCTION_DUTY_CYCLE_ANGLE 0.85 // not necessarily min... but close enough DON"T MAKE THIS SERVO_MOUNTING_ANGLE OR THE MAX STEER EITHER
#define DUTY_CYCLE_REDUCTION_OUTER 3
#define DUTY_CYCLE_REDUCTION_INNER 15


// angle tolerance for ending a gyro turn
#define ANGLE_TOLERANCE_RADIANS 0.04

// servo steering
#define SERVO_MOUNTING_ANGLE 1.55
#define SERVO_MAX_STEER 0.35

// sonar
#define SONAR_FREQUENCY 17 // 1/60ms = 17Hz
#define BRIDGE_DISTANCE 20
#define WALL_DISTANCE 20

// accelerometer falling threshold
#define FALLING_ACCELERATION 3

// OLED
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int OLED_RESET = -1; 