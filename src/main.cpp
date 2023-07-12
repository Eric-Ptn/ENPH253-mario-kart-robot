#include <Arduino.h>
#include <servo_steering.h>

Servo servo;

void setup() {
  servo.attach(SERVO_PIN);
  for (int i = 0; i < NUM_IR_SENSORS; i++)  {
      pinMode(IR_PINS[i], INPUT);
  }
}

void loop() {
  steer(servo);
}
