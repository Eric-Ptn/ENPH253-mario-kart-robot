#pragma once  // avoid circular inclusion
#include <Arduino.h>

// pins
#define SERVO_PIN PA6
#define SERVO_PIN_PWM_NAME PA_6

#define LEFT_MOTOR_PIN PA8
#define LEFT_MOTOR_PIN_PWM_NAME PA_8

#define RIGHT_MOTOR_PIN PA9
#define RIGHT_MOTOR_PIN_PWM_NAME PA_9


// IR sensor pins
const byte IR_PINS[] = {PA1, PA2, PA3, PA4};
const int NUM_IR_SENSORS = sizeof(IR_PINS) / sizeof(IR_PINS[0]);

// steering PID values
// get this to map to potentiometer values later for tuning?
#define Kp 50
#define Ki 0
#define Kd 0

// gyro calibration values
#define FAST_CALIBRATION_RUNS 5000
#define SLOW_CALIBRATION_SECONDS 15

// PWM frequencies
#define SERVO_FREQUENCY_HZ 50 // THIS SHOULD NOT CHANGE
#define MOTOR_FREQUENCY_HZ 1000 // this can change

// motor speeds
#define DEFAULT_MOTOR_DUTY_CYCLE 100
#define MOTOR_CORRECTION_SCALING 2 // arbitrary, find experimentally

// angle tolerance
#define ANGLE_TOLERANCE_RADIANS 0.04

// turning
#define PROPORTIONAL_STEERING 1
