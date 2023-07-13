#pragma once  // avoid circular inclusion
#include <Arduino.h>

// pins
#define SERVO_PIN PA6
#define SERVO_PIN_PWM_NAME PA_6

// IR sensor pins
const byte IR_PINS[] = {PA1, PA2, PA3, PA4};
const int NUM_IR_SENSORS = sizeof(IR_PINS) / sizeof(IR_PINS[0]);

// steering PID values
// get this to map to potentiometer values later for tuning?
const int Kp = 50;
const int Ki = 0;
const int Kd = 0;