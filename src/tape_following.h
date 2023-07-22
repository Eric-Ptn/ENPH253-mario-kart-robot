#include <Arduino.h>
#include <Wire.h>
#include <global_values.h>
#include <pwm_driving.h>
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
static double max_integral = 1.5; // wind-up safety
static int ir_readings[NUM_IR_SENSORS];
static double desired_center = NUM_IR_SENSORS / 2 + 0.5; // position indices in weighted average start from 1, so add 0.5 to get the center
static int ir_offsets[NUM_IR_SENSORS] = {0};
static double ir_scaling[NUM_IR_SENSORS];

void constant_offset_tape_sensors() {
  int sensor_sums[NUM_IR_SENSORS] = {0};

  for(int i = 0; i < IR_CALIBRATION_RUNS; i++) {
    for(int j = 0; j < NUM_IR_SENSORS; j++) {
      sensor_sums[j] += analogRead(IR_PINS[j]);
    }
  }

  for(int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_offsets[i] = sensor_sums[i] / IR_CALIBRATION_RUNS - WHITE_THRESHOLD;
  }

  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_scaling[i] = 1;
  }
}

void scaling_offset_tape_sensors() {
  int sensor_sums[NUM_IR_SENSORS] = {0};

  for(int i = 0; i < IR_CALIBRATION_RUNS; i++) {
    for(int j = 0; j < NUM_IR_SENSORS; j++) {
      sensor_sums[j] += analogRead(IR_PINS[j]);
    }
  }

  // find max ir reading, scale each sensor to same sensitivity

  int sensor_averages[NUM_IR_SENSORS] = {0};
  double highest_average = sensor_sums[0] / IR_CALIBRATION_RUNS;
  for(int i = 0; i < NUM_IR_SENSORS; i++) {
    sensor_averages[i] = sensor_sums[i] / IR_CALIBRATION_RUNS;
    if (sensor_averages[i] > highest_average) {
      highest_average = sensor_averages[i];
    }
  }

  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_scaling[i] = highest_average / sensor_averages[i];
    ir_offsets[i] = highest_average - WHITE_THRESHOLD;
  }

}

// steering PID values
// get this to map to potentiometer values later for tuning?
static double Kp = 0.2;
static double Ki = 0;
static double Kd = 0.05;

bool seeing_white() {
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_readings[i] = analogRead(IR_PINS[i]) * ir_scaling[i] - ir_offsets[i];
    if (ir_readings[i] < 0) {
      return false;
    }
  }

  return true;
}

bool seeing_black() {
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_readings[i] = analogRead(IR_PINS[i]) * ir_scaling[i] - ir_offsets[i];
    if (ir_readings[i] < -1 * WHITE_THRESHOLD) {
      return false;
    }
  }
  
  return true;
}

// give processed ir reading for an IR tape sensor pin, zero-indexed
double processed_ir_reading(int i) {
  for(int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_readings[i] = analogRead(IR_PINS[i]) - ir_offsets[i];
  }
  return min(int(analogRead(IR_PINS[i])) - ir_offsets[i], 0);
}


// void test_sensors() {
//   // raw analog readings
//   for(int i = 0; i < NUM_IR_SENSORS; i++){
//     ir_readings[i] = analogRead(IR_PINS[i]);
//   }

//   // OLED display, feel free to comment out
//   String servo_info = "";
//   for(int i = 0; i < NUM_IR_SENSORS; i++) {
//     servo_info += " Sensor " + String(i) + ": " + String(ir_readings[i] - ir_offsets[i]);
//   }

//   display_text(servo_info);
// }

/*
Purpose: PID routine to follow tape. 
*/
void tape_follow_drive() {

  // raw analog readings
  for(int i = 0; i < NUM_IR_SENSORS; i++){
    ir_readings[i] = analogRead(IR_PINS[i]);
  }

  // do weighted average to find current center of robot, ASSUMES EQUAL SPACING BETWEEN SENSORS
  double current_position = 0;
  double sum_of_weights = 0;
  for(int i = 1; i <= NUM_IR_SENSORS; i++){  // cannot be zero-indexed because it screws up the weighted average
    if (ir_readings[i - 1] - ir_offsets[i - 1] > 0) {
      continue;
    } else {
      current_position += (ir_readings[i - 1] * ir_scaling[i - 1] - ir_offsets[i - 1]) * i; // careful here! i is 1-indexed, ir_readings is 0-indexed
      sum_of_weights += ir_readings[i - 1] * ir_scaling[i - 1] - ir_offsets[i - 1];
    }
  }

  current_position /= sum_of_weights; // a decimal from 1 to NUM_IR_SENSORS representing the current position of the tape relative to robot
  double error = desired_center - current_position; // (ranges from 0 to desired_center - 1)
  
  // compute PID components
  proportional = error;

  if (abs(integral + error) < max_integral) {
    integral += error;
  } else if (integral + error < 0) {
    integral = -1 * max_integral;
  } else {
    integral = max_integral;
  }

  // integral = min(integral + error, max_integral);
  derivative = error - last_error;
  last_error = error;

  double correction_val = Kp * proportional + Kd * derivative + Ki * integral;
  // double correction_val = 1;

  servo_pwm(SERVO_MOUNTING_ANGLE + correction_val);

  // OLED display, feel free to comment out
  String servo_info = "";
  // String servo_info = "Servo write: " + String(SERVO_MOUNTING_ANGLE - correction_val) + " correction value: " + String(correction_val) + " error: " + String(error);
  for(int i = 0; i < NUM_IR_SENSORS; i++) {
    if (ir_readings[i] * ir_scaling[i] - ir_offsets[i] > 0) {
      servo_info += ", Sensor " + String(i) + ": 0";
    } else {
      servo_info += ", Sensor " + String(i) + ": " + String(ir_readings[i] * ir_scaling[i] - ir_offsets[i]);
    }
  }

  display_text(servo_info);

  // drive motors, slow down based on how far off you are (ex. turn)
  left_motor_steering_drive(SERVO_MOUNTING_ANGLE + correction_val, false);
  right_motor_steering_drive(SERVO_MOUNTING_ANGLE + correction_val, false);
}