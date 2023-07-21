#pragma once  // avoid circular inclusion
#include <Arduino.h>
#include<math.h>

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

// gyro and OLED connect to I2C pins, PB6 and PB7


// IR sensor pins
const byte IR_PINS[] = {PA0, PA1, PA3, PA4};
const int NUM_IR_SENSORS = sizeof(IR_PINS) / sizeof(IR_PINS[0]);

// gyro calibration values
#define GYRO_FAST_CALIBRATION_RUNS 5000
#define GYRO_SLOW_CALIBRATION_SECONDS 15

// ir calibration
#define IR_CALIBRATION_RUNS 5000
#define WHITE_THRESHOLD 75

// PWM frequencies
#define SERVO_FREQUENCY_HZ 50 // THIS SHOULD NOT CHANGE
#define MOTOR_FREQUENCY_HZ 100 // this can change within reason - should not start hearing high-pitched whine

// motor speeds
#define DEFAULT_MOTOR_DUTY_CYCLE 50
#define MOTOR_CORRECTION_SCALING 20 // arbitrary, find experimentally

// angle tolerance for ending a gyro turn
#define ANGLE_TOLERANCE_RADIANS 0.04

// mounting angle for servo
#define SERVO_MOUNTING_ANGLE M_PI / 2

// sonar
#define SONAR_FREQUENCY 17 // 1/60ms = 17Hz
#define BRIDGE_DISTANCE 20
#define WALL_DISTANCE 20
