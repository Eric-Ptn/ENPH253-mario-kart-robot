#pragma once  // avoid circular inclusion
#include <Arduino.h>

// pins
#define SERVO_PIN 9 

// IR sensor pins
const byte IR_PINS[] = {A0, A1, A2, A3};
const int NUM_IR_SENSORS = sizeof(IR_PINS) / sizeof(IR_PINS[0]);

// steering PID values
// get this to map to potentiometer values later for tuning?
const int Kp = 5;
const int Ki = 0;
const int Kd = 0;