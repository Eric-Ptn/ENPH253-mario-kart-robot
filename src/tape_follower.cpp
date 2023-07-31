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

  if (ir_processed < WHITE_THRESHOLD) {
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
    ir_offsets[i] = sensor_sums[i] / IR_CALIBRATION_RUNS;
  }
}


void TapeFollower::scaling_offset_calibration() {
  int sensor_sums[NUM_IR_SENSORS] = {0};

  for(int i = 0; i < IR_CALIBRATION_RUNS; i++) {
    for(int j = 0; j < NUM_IR_SENSORS; j++) {
      sensor_sums[j] += analogRead(IR_PINS[j]);
      // OLED::display_text(String(sensor_sums[j]));
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
    ir_offsets[i] = highest_average;
  }

  OLED::display_text("s0 scaling: " + String(ir_scaling[0]) + " s0 offset: " + String(ir_offsets[0]) + 
  " s1 scaling: " + String(ir_scaling[1]) + " s1 offset: " + String(ir_offsets[1]) + " s2 scaling: " + String(ir_scaling[2]) + " s2 offset: " + String(ir_offsets[2])
  + " s3 scaling: " + String(ir_scaling[3]) + " s3 offset: " + String(ir_offsets[3]));

  // all offsets are the same and positive, scaling is above 1

}

void TapeFollower::quick_calibration() {
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_offsets[i] = SAVED_IR_OFFSETS[i];
    ir_scaling[i] = SAVED_IR_SCALING[i];
  }
}

void TapeFollower::tape_calibration() {

  double ir_mins[NUM_IR_SENSORS];
  std::fill(ir_mins, ir_mins + NUM_IR_SENSORS, 1023);

  double ir_maxs[NUM_IR_SENSORS];


  motors::left_motor_PWM(25);
  motors::right_motor_PWM(25);

  double start_time = millis();
  while (millis() - start_time < 5000) {

    for (int i = 0; i < NUM_IR_SENSORS; i++) {
      int ir_reading = analogRead(IR_PINS[i]);

      if (ir_reading > ir_maxs[i]) {
        ir_maxs[i] = ir_reading;
      }
      if (ir_reading < ir_mins[i]) {
        ir_mins[i] = ir_reading;
      }

    }

  }

  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_scaling[i] = (WHITE_VALUE - BLACK_VALUE) / (ir_maxs[i] - ir_mins[i]); // slope of line
    ir_offsets[i] = ir_scaling[i] * ir_maxs[i] - WHITE_VALUE; // y-intercept of line, b = y - mx, but negative because i subtract offsets
  }

  motors::left_motor_PWM(0);
  motors::right_motor_PWM(0);

  OLED::display_text("s0: " + String(ir_scaling[0]) + ", " + String(ir_offsets[0]) + ", " + String(ir_maxs[0]) + ", " + String(ir_mins[0]) +
  " s1: " + String(ir_scaling[1]) + ", " + String(ir_offsets[1]) + ", " + String(ir_maxs[1]) + ", " + String(ir_mins[1]) +
  " s2: " + String(ir_scaling[2]) + ", " + String(ir_offsets[2]) + ", " + String(ir_maxs[2]) + ", " + String(ir_mins[2]) +
  " s3: " + String(ir_scaling[3]) + ", " + String(ir_offsets[3]) + ", " + String(ir_maxs[3]) + ", " + String(ir_mins[3]));

}


void TapeFollower::follow_tape(double duty_cycle_offset) {

  // do weighted average to find current center of robot, ASSUMES EQUAL SPACING BETWEEN SENSORS
  double current_position = 0;
  double sum_of_weights = 0;
  for(int i = 1; i <= NUM_IR_SENSORS; i++){  // cannot be zero-indexed because it screws up the weighted average
    current_position += processed_ir_reading(i - 1) * i; // careful here! i is 1-indexed, ir_readings is 0-indexed
    sum_of_weights += processed_ir_reading(i - 1);
  }

  double error;
  if (sum_of_weights < ERROR_MEMORY_THRESHOLD) {        // use to have another negative offset - if not all black, calculate error else use old error
    current_position /= sum_of_weights;                 // a decimal from 1 to NUM_IR_SENSORS representing the current position of the tape relative to robot
    error = desired_center - current_position;          // (ranges from 0 to desired_center - 1)
    // OLED::display_text("Normal error: "+ String(error));
  } else {
    // if you don't see anything, assume the error was similar to before, and turn HARD in that direction
    // unfortunately this is quite a banana prone implementation, may change later to just previous error
    if (last_error < 0) {
      error = -1 * SERVO_MAX_STEER;
    } else {
      error = SERVO_MAX_STEER; 
    }
    current_position = -1;
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

  double servo_angle = SERVO_MOUNTING_ANGLE + correction_val;
  if (servo_angle > SERVO_MOUNTING_ANGLE + SERVO_MAX_STEER) {
      servo_angle = SERVO_MOUNTING_ANGLE + SERVO_MAX_STEER;
  } else if (servo_angle < SERVO_MOUNTING_ANGLE - SERVO_MAX_STEER) {
      servo_angle = SERVO_MOUNTING_ANGLE - SERVO_MAX_STEER;
  }

  motors::servo_pwm(servo_angle);

  // OLED display, feel free to comment out
  // String servo_info = "Servo write: " + String(servo_angle) + " correction value: " + String(correction_val) + " position: " + String(current_position);
  // for(int i = 0; i < NUM_IR_SENSORS; i++) {
  //   servo_info += ", Sensor " + String(i) + ": " + String(processed_ir_reading(i));
  // }

  // OLED::display_text(servo_info);

  motors::left_motor_steering_drive(servo_angle, false);
  motors::right_motor_steering_drive(servo_angle, false);
}


// put this in other function?
bool TapeFollower::seeing_white() {
  double sum_readings = 0;
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    sum_readings += processed_ir_reading(i);
  }

  if (sum_readings < ERROR_MEMORY_THRESHOLD) {
    return false;
  }

  return true;
}


bool TapeFollower::seeing_black() {
  return !seeing_white();
}
