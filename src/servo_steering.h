#include <Arduino.h>
#include <Servo.h>
#include <global_values.h>

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
static double last_time = micros(); // millis returns unsigned long be careful?

void steer(Servo servo) {
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
  double dt = micros() - last_time;

  proportional = error;
  integral = min(integral + error * dt, max_integral);
  derivative = (last_error - error) / dt;
  last_error = error;

  double correction_val = Kp * proportional + Ki * integral + Kd * derivative;

  // assumes that the servo is mounted at 90 degrees 
  servo.write(90 - correction_val);

  last_time = micros();
}
