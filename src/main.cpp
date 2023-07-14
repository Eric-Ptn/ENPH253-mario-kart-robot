#include <Arduino.h>
#include <tape_following.h>
#include <global_values.h>
#include <gyro.h>
#include <sonar.h>

void setup() {
  // // pins
  // pinMode(SERVO_PIN, OUTPUT);
  // for (int i = 0; i < NUM_IR_SENSORS; i++)  {
  //     pinMode(IR_PINS[i], INPUT);
  // }

  pinMode(SONAR_TRIGGER_PIN_1, OUTPUT);
  pinMode(SONAR_ECHO_PIN_1, INPUT);

  // // i2c adafruit components
  begin_oled();
  // begin_gyro();

  // // gyro calibration
  // velocity_calibrate();
  // display_text("fast calibration complete!");
  // delay(2000);

  // slow_calibrate();
  // display_text("slow calibration complete!");
  // delay(2000);

  trigger_sonar(1);
}

void loop() {
  
  long distance = measure_distance(1);

  display_text("Distance: " + String(distance));

  delay(10);
}
