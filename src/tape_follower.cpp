#include <Arduino.h>
#include <Wire.h>
#include <mario-kart-robot\tape_follower.h>
#include <mario-kart-robot\motors.h>
#include <mario-kart-robot\config.h>
#include <mario-kart-robot\oled_display.h>


TapeFollower::TapeFollower() {
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_scaling[i] = 1;
  }
}


// give processed ir reading for an IR tape sensor pin, zero-indexed
int TapeFollower::processed_ir_reading(int i) {

  int ir_processed = analogRead(IR_PINS[i]) * ir_scaling[i] - ir_offsets[i];

  if (ir_processed < 0) {
    return ir_processed;
  } else {
    return 0;
  }
}


void TapeFollower::constant_offset_calibration() {
  int sensor_sums[NUM_IR_SENSORS];

  for(int i = 0; i < IR_CALIBRATION_RUNS; i++) {
    for(int j = 0; j < NUM_IR_SENSORS; j++) {
      sensor_sums[j] += analogRead(IR_PINS[j]);
    }
  }

  for(int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_offsets[i] = sensor_sums[i] / IR_CALIBRATION_RUNS - WHITE_THRESHOLD;
  }
}


void TapeFollower::scaling_offset_calibration() {
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


void TapeFollower::follow_tape() {

  // do weighted average to find current center of robot, ASSUMES EQUAL SPACING BETWEEN SENSORS
  double current_position = 0;
  double sum_of_weights = 0;
  for(int i = 1; i <= NUM_IR_SENSORS; i++){  // cannot be zero-indexed because it screws up the weighted average
    current_position += processed_ir_reading(i - 1) * i; // careful here! i is 1-indexed, ir_readings is 0-indexed
    sum_of_weights += processed_ir_reading(i - 1);
  }

  double error;
  if (sum_of_weights < SAME_ERROR_THRESHOLD) { // use to have another negative offset - if not all black, calculate error else use old error
    current_position /= sum_of_weights; // a decimal from 1 to NUM_IR_SENSORS representing the current position of the tape relative to robot
    error = desired_center - current_position; // (ranges from 0 to desired_center - 1)
    // OLED::display_text("Normal error: "+ String(error));
  } else {
    if (last_error < 0) {
      error = -1 * SERVO_MAX_STEER;
    } else {
      error = SERVO_MAX_STEER; // if you don't see anything, assume the error was similar to before, avoid division by zero problem as well
    }
    // OLED::display_text("Remembered error: "+ String(error));
  }

  
  // compute PID components
  proportional = error;

  if (abs(integral + error) < max_integral) {
    integral += error;
  } else if (integral + error < 0) {
    integral = -1 * max_integral;
  } else {
    integral = max_integral;
  }

  derivative = error - last_error;
  last_error = error;

  double correction_val = Kp * proportional + Kd * derivative + Ki * integral;

  motors::servo_pwm(SERVO_MOUNTING_ANGLE + correction_val);

  // OLED display, feel free to comment out
  // String servo_info = "Servo write: " + String(SERVO_MOUNTING_ANGLE - correction_val) + " correction value: " + String(correction_val) + " position: " + String(current_position);
  // for(int i = 0; i < NUM_IR_SENSORS; i++) {
  //   servo_info += ", Sensor " + String(i) + ": " + String(processed_ir_reading(i));
  // }

  // OLED::display_text(servo_info);

  // drive motors, slow down based on how far off you are (ex. turn)
  motors::left_motor_steering_drive(SERVO_MOUNTING_ANGLE + correction_val, false);
  motors::right_motor_steering_drive(SERVO_MOUNTING_ANGLE - correction_val, false);
}



bool TapeFollower::seeing_white() {
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    if (processed_ir_reading(i) < 0) {
      return false;
    }
  }

  return true;
}


bool TapeFollower::seeing_black() {
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    if (processed_ir_reading(i) == 0) {
      return false;
    }
  }
  
  return true;
}

// bool TapeFollower::test_bool(){
//   if (millis() > 35000) {
//     return true;
//   }
//   return false;
// }