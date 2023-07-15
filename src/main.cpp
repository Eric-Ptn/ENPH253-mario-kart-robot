#include <Arduino.h>
#include <tape_following.h>
#include <global_values.h>
#include <gyro.h>

void setup() {
  // pins
  pinMode(SERVO_PIN, OUTPUT);
  for (int i = 0; i < NUM_IR_SENSORS; i++)  {
      pinMode(IR_PINS[i], INPUT);
  }

  // i2c adafruit components
  begin_oled();
  // begin_gyro();

  // gyro calibration
  // velocity_calibrate();
  // display_text("fast calibration complete!");

  // delay(2000);

  // slow_calibrate();
  // display_text("slow calibration complete!");
  // delay(2000);

  // ir calibration
  calibrate_tape_sensors();
}

void loop() {
  tape_follow_drive();
}
