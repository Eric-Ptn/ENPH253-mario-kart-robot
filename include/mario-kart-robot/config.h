#pragma once  // avoid circular inclusion
#include <Arduino.h>
#include <math.h>
#include <Adafruit_SSD1306.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>

// TODO: add PID constants

// pins
#define SERVO_PIN PA6
#define SERVO_PIN_PWM_NAME PA_6

#define LEFT_MOTOR_PIN PB6
#define LEFT_MOTOR_PIN_PWM_NAME PB_6
#define LEFT_REVERSE_MOTOR_PIN PB7
#define LEFT_REVERSE_MOTOR_PIN_PWM_NAME PB_7

#define RIGHT_MOTOR_PIN PB8
#define RIGHT_MOTOR_PIN_PWM_NAME PB_8
#define RIGHT_REVERSE_MOTOR_PIN PB9
#define RIGHT_REVERSE_MOTOR_PIN_PWM_NAME PB_9

#define BRIDGE_SONAR_TRIGGER PA8
#define BRIDGE_SONAR_PWM_NAME PA_8
#define BRIDGE_SONAR_ECHO PA11

#define WALL_SONAR_TRIGGER PA9
#define WALL_SONAR_PWM_NAME PA_9  
#define WALL_SONAR_ECHO PA12

// #define CALIBRATION_PIN PB3
// #define RUNNING_PIN PB4

// gyro and OLED connect to 2nd I2C pins, PB10 and PB11


// IR sensor pins
const byte IR_PINS[] = {PA1, PA0, PA4, PA3};
const int NUM_IR_SENSORS = sizeof(IR_PINS) / sizeof(IR_PINS[0]);

// gyro PID values
#define GYRO_KP 1.1
#define GYRO_KI 0
#define GYRO_KD 0.7
#define GYRO_MAX_INTEGRAL 0

// tape following PID values
#define TAPE_FOLLOWING_KP 3
#define TAPE_FOLLOWING_KI 0.25
#define TAPE_FOLLOWING_KD 1
#define TAPE_FOLLOWING_MAX_INTEGRAL 1

// gyro calibration values
#define IMU_FAST_CALIBRATION_RUNS 5000
#define IMU_SLOW_CALIBRATION_SECONDS 15

// ir calibration
#define IR_CALIBRATION_RUNS 5000
#define WHITE_THRESHOLD 175
#define SAME_ERROR_THRESHOLD -75

// PWM frequencies (https://components101.com/motors/mg996r-servo-motor-datasheet)
#define SERVO_FREQUENCY_HZ 50 // THIS SHOULD NOT CHANGE
#define MOTOR_FREQUENCY_HZ 100 // this can change within reason - should not start hearing high-pitched whine

// motor speeds
#define DEFAULT_MOTOR_DUTY_CYCLE 25
#define MOTOR_CORRECTION_SCALING 4 // arbitrary, find experimentally
// #define MOTOR_CORRECTION_SCALING_INNER 2.5
// #define MOTOR_CORRECTION_SCALING_OUTER 1.5
#define CORRECTION_VAL_THRESHOLD 1 // if correction_val < CORRECTION_VAL_THRESHOLD then the scaling is the same for both motors

// angle tolerance for ending a gyro turn
#define ANGLE_TOLERANCE_RADIANS 0.04

// servo steering
// #define SERVO_MOUNTING_ANGLE M_PI/6
#define SERVO_MOUNTING_ANGLE 0.5
#define SERVO_MAX_STEER 0.5

// sonar
#define SONAR_FREQUENCY 17 // 1/60ms = 17Hz
#define BRIDGE_DISTANCE 20
#define WALL_DISTANCE 20

// accelerometer falling threshold
#define FALLING_ACCELERATION 3

// OLED
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int OLED_RESET = -1; // This display does not have a reset pin accessible