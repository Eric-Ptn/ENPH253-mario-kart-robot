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

  // int multiple = 50;
  // int ir_processed = (analogRead(IR_PINS[i]) * ir_scaling[i] - ir_offsets[i]) + multiple/2;
  // ir_processed -= ir_processed % multiple;
  int ir_processed = analogRead(IR_PINS[i]) * ir_scaling[i] - ir_offsets[i];

  if (ir_processed < WHITE_THRESHOLD) {
    return ir_processed;
  } else {
    return 0;
  }
}

int TapeFollower::ir_reading_no_threshold(int i) {
  return analogRead(IR_PINS[i]) * ir_scaling[i] - ir_offsets[i];
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
  double lowest_average = sensor_sums[0] / IR_CALIBRATION_RUNS;
  for(int i = 0; i < NUM_IR_SENSORS; i++) {
    sensor_averages[i] = sensor_sums[i] / IR_CALIBRATION_RUNS;
    if (sensor_averages[i] < lowest_average) {
      lowest_average = sensor_averages[i];
    }
  }

  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_scaling[i] = lowest_average / sensor_averages[i];
    ir_offsets[i] = lowest_average;
  }

  // OLED::display_text("s0 scaling: " + String(ir_scaling[0]) + " s0 offset: " + String(ir_offsets[0]) + 
  // " s1 scaling: " + String(ir_scaling[1]) + " s1 offset: " + String(ir_offsets[1]) + " s2 scaling: " + String(ir_scaling[2]) + " s2 offset: " + String(ir_offsets[2])
  // + " s3 scaling: " + String(ir_scaling[3]) + " s3 offset: " + String(ir_offsets[3]) + "\n");

  // all offsets are the same and positive, scaling is above 1

}

void TapeFollower::quick_calibration() {
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_offsets[i] = SAVED_IR_OFFSETS[i];
    ir_scaling[i] = SAVED_IR_SCALING[i];
  }
}

void TapeFollower::tape_calibration() {

  double ir_white_sums[NUM_IR_SENSORS];
  double ir_black_sums[NUM_IR_SENSORS];
  
  double ir_whites[NUM_IR_SENSORS];
  double ir_blacks[NUM_IR_SENSORS];

  for(int i = 0; i < IR_CALIBRATION_RUNS; i++) {
    for(int j = 0; j < NUM_IR_SENSORS; j++) {
      ir_white_sums[j] += analogRead(IR_PINS[j]);
    }
  }

  OLED::display_text("move to black...");
  delay(3000);
  OLED::display_text(String(ir_white_sums[1]));
  delay(2000);

  for(int i = 0; i < IR_CALIBRATION_RUNS; i++) {
    for(int j = 0; j < NUM_IR_SENSORS; j++) {
      ir_black_sums[j] += analogRead(IR_PINS[j]);
    }
  }
  OLED::display_text(String(ir_black_sums[1]));
  delay(2000);
  

  for(int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_whites[i] = ir_white_sums[i] / IR_CALIBRATION_RUNS;
    ir_blacks[i] = ir_black_sums[i] / IR_CALIBRATION_RUNS;
  }

  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    ir_scaling[i] = (WHITE_VALUE - BLACK_VALUE) / (ir_whites[i] - ir_blacks[i]); // slope of line
    ir_offsets[i] = ir_scaling[i] * ir_whites[i] - WHITE_VALUE; // y-intercept of line, b = y - mx, but negative because i subtract offsets
  }

  OLED::display_text("s0: " + String(ir_scaling[0]) + ", " + String(ir_offsets[0]) + ", " + String(ir_whites[0]) + ", " + String(ir_blacks[0]) +
  " s1: " + String(ir_scaling[1]) + ", " + String(ir_offsets[1]) + ", " + String(ir_whites[1]) + ", " + String(ir_blacks[1]) +
  " s2: " + String(ir_scaling[2]) + ", " + String(ir_offsets[2]) + ", " + String(ir_whites[2]) + ", " + String(ir_blacks[2]) +
  " s3: " + String(ir_scaling[3]) + ", " + String(ir_offsets[3]) + ", " + String(ir_whites[3]) + ", " + String(ir_blacks[3]));

}


void TapeFollower::follow_tape(double duty_cycle_offset) {

  // return if you're spamming PWM too fast
  // if (millis() - last_motor_time < 1000 / SERVO_FREQUENCY_HZ) {
  //   return;
  // }

  // do weighted average to find current center of robot, ASSUMES EQUAL SPACING BETWEEN SENSORS
  double current_position = 0;
  double sum_of_weights = 0;
  for(int i = 1; i <= NUM_IR_SENSORS; i++){  // cannot be zero-indexed because it screws up the weighted average
    double processed_reading = processed_ir_reading(i - 1); // careful here! i is 1-indexed, ir_readings is 0-indexed
    current_position += processed_reading * i; 
    sum_of_weights += processed_reading;
  }

  double error;
  if (sum_of_weights < ERROR_MEMORY_THRESHOLD) {        // use to have another negative offset - if not all black, calculate error else use old error
    current_position /= sum_of_weights;                 // a decimal from 1 to NUM_IR_SENSORS representing the current position of the tape relative to robot
    error = desired_center - current_position;          // (ranges from 0 to desired_center - 1, positive or negative)
    // OLED::display_text("Normal error: "+ String(error));
  } else {
    // if you don't see anything, assume the error was similar to before, and turn HARD in that direction
    // unfortunately this is quite a banana prone implementation, may change later to just previous error
    // if (last_error < 0) {
    //   error = -1 * (desired_center - 1);
    // } else {
    //   error = desired_center - 1; 
    // }
    error = last_error;
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

  // String text = "s0: " + String(processed_ir_reading(0)) + 
  //                 " s1: " + String(processed_ir_reading(1)) +
  //                 " s2: " + String(processed_ir_reading(2)) +
  //                 " s3: " + String(processed_ir_reading(3)) + " error: " + String(error) + "\n";

  // Serial.print(text);

  motors::servo_pwm(servo_angle);

  // OLED display, feel free to comment out
  String servo_info = "Servo write: " + String(servo_angle) + " correction value: " + String(correction_val) + " position: " + String(current_position);
  // for(int i = 0; i < NUM_IR_SENSORS; i++) {
  //   servo_info += ", Sensor " + String(i) + ": " + String(processed_ir_reading(i));
  // }

  OLED::display_text(servo_info);

  double dumb_angle = SERVO_MOUNTING_ANGLE - correction_val;
  if (dumb_angle > SERVO_MOUNTING_ANGLE + SERVO_MAX_STEER) {
      dumb_angle = SERVO_MOUNTING_ANGLE + SERVO_MAX_STEER;
  } else if (dumb_angle < SERVO_MOUNTING_ANGLE - SERVO_MAX_STEER) {
      dumb_angle = SERVO_MOUNTING_ANGLE - SERVO_MAX_STEER;
  }

  motors::left_motor_steering_drive(dumb_angle, false);
  motors::right_motor_steering_drive(dumb_angle, false);

  // last_motor_time = millis();

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
