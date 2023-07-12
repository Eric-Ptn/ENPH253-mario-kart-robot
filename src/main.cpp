#include <Arduino.h>
#include <servo_steering.h>
#include <global_values.h>

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  for (int i = 0; i < NUM_IR_SENSORS; i++)  {
      pinMode(IR_PINS[i], INPUT);
  }
}

void loop() {
  tape_follow();
}
