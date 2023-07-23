#pragma once  // avoid circular inclusion
#include <Arduino.h>
#include <math.h>

// TODO: add PID constants

// pins
#define SERVO_PIN PB9
#define SERVO_PIN_PWM_NAME PB_9

#define LEFT_MOTOR_PIN PB0
#define LEFT_MOTOR_PIN_PWM_NAME PB_0
#define LEFT_REVERSE_MOTOR_PIN PB1
#define LEFT_REVERSE_MOTOR_PIN_PWM_NAME PB_1

#define RIGHT_MOTOR_PIN PA6
#define RIGHT_MOTOR_PIN_PWM_NAME PA_6
#define RIGHT_REVERSE_MOTOR_PIN PA7
#define RIGHT_REVERSE_MOTOR_PIN_PWM_NAME PA_7

#define BRIDGE_SONAR_TRIGGER PA8
#define BRIDGE_SONAR_PWM_NAME PA_8
#define BRIDGE_SONAR_ECHO PA11

#define WALL_SONAR_TRIGGER PA9
#define WALL_SONAR_PWM_NAME PA_9  
#define WALL_SONAR_ECHO PA12

#define CALIBRATION_PIN PB3
#define RUNNING_PIN PB4

// gyro and OLED connect to I2C pins, PB6 and PB7


// IR sensor pins
const byte IR_PINS[] = {PA1, PA0, PA4, PA3};
const int NUM_IR_SENSORS = sizeof(IR_PINS) / sizeof(IR_PINS[0]);

// gyro PID values
#define GYRO_KP 0
#define GYRO_KI 0
#define GYRO_KD 0
#define GYRO_MAX_INTEGRAL 0

// tape following PID values
#define TAPE_FOLLOWING_KP 0.2
#define TAPE_FOLLOWING_KI 0
#define TAPE_FOLLOWING_KD 0
#define TAPE_FOLLOWING_MAX_INTEGRAL 0

// gyro calibration values
#define GYRO_FAST_CALIBRATION_RUNS 5000
#define GYRO_SLOW_CALIBRATION_SECONDS 15

// ir calibration
#define IR_CALIBRATION_RUNS 5000
#define WHITE_THRESHOLD 100

// PWM frequencies (https://components101.com/motors/mg996r-servo-motor-datasheet)
#define SERVO_FREQUENCY_HZ 50 // THIS SHOULD NOT CHANGE
#define MOTOR_FREQUENCY_HZ 100 // this can change within reason - should not start hearing high-pitched whine

// motor speeds
#define DEFAULT_MOTOR_DUTY_CYCLE 30
#define MOTOR_CORRECTION_SCALING 10 // arbitrary, find experimentally

// angle tolerance for ending a gyro turn
#define ANGLE_TOLERANCE_RADIANS 0.04

// mounting angle for servo
#define SERVO_MOUNTING_ANGLE M_PI/6

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