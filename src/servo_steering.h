#include <Arduino.h>
#include <Wire.h>
#include <global_values.h>
#include <oled_display.h>

/*
This is just an initial attempt to code the control system for tape following (I would try to make this better but sick)
Potential improvements:
- Add a history for the error (perhaps an array that has previous errors, if the error is increasing, steer harder)
   - Eric's ridicule: i thought about this too, but i think this is effectively just increasing integral control

- Maybe alter this to work with only 2 IR sensors (maybe this would make the code run faster, but honestly, I don't know if it will
  matter much)
   - Eric's ridicule: code should now work for any number of IR sensors
*/

// variables for PID control, "static" ensures variables only have local file scope
static double proportional = 0;
static double integral = 0;
static double derivative = 0;
static double last_error = 0;
static double max_integral = 100; // wind-up safety
static int ir_readings[NUM_IR_SENSORS];
static double desired_center = NUM_IR_SENSORS / 2 + 0.5; // position indices in weighted average start from 1, so add 0.5 to get the center

void tape_follow() {
  // analog readings
  for(int i = 0; i < NUM_IR_SENSORS; i++){
    ir_readings[i] = analogRead(IR_PINS[i]);
  }

  // do weighted average to find current center of robot, ASSUMES EQUAL SPACING BETWEEN SENSORS
  double current_position = 0;
  int sum_of_weights = 0;
  for(int i = 1; i <= NUM_IR_SENSORS; i++){  // cannot be zero-indexed because it screws up the weighted average
    current_position += ir_readings[i - 1] * i; // careful here! i is 1-indexed, ir_readings is 0-indexed
    sum_of_weights += ir_readings[i - 1];
  }

  current_position /= sum_of_weights; // a decimal from 1 to NUM_IR_SENSORS representing the current position of the tape relative to robot
  double error = current_position - desired_center;
  
  // compute PID components
  proportional = error;
  integral = min(integral + error, max_integral);
  derivative = last_error - error;
  last_error = error;

  double correction_val = Kp * proportional + Ki * integral + Kd * derivative;

  // assumes that the servo is mounted at 90 degrees 
  moving_servo(90 - correction_val);

  // OLED display, feel free to comment out
  String servo_info = "Servo write: " + String(90 - correction_val);
  for(int i = 0; i < NUM_IR_SENSORS; i++) {
    servo_info += ", Sensor " + String(i) + ": " + String(ir_readings[i]);
  }

  display_text(servo_info);
}

void moving_servo(double angle) {

  // ~0 degrees is 2% duty cycle, ~180 degrees is 10% duty cycle
  // servo.h does NOT work nicely, use this instead
  double duty_cycle = (10 - 2)/(180 - 0) * (angle - 0) + 2;
  int frequency_Hz = 50; // specified in servo datasheet

  pwm_start(SERVO_PIN_PWM_NAME, frequency_Hz, duty_cycle, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
}
