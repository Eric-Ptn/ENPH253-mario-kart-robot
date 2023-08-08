#include <mario-kart-robot\imu.h>
#include <mario-kart-robot\oled_display.h>
#include <mario-kart-robot\motors.h>
#include <cmath>

void IMU::begin_imu() {
    imu.begin();
}

// void IMU::i2c_reboot() {
//   Wire.beginTransmission(MPU6050_I2CADDR_DEFAULT);
//   Wire.write(0x43);
//   if(Wire.endTransmission() != 0) {
//     imu.begin();
//   }
// }

void IMU::read_imu() {
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  gyro_readings[0] = g.gyro.x - gyro_offsets[0];
  gyro_readings[1] = g.gyro.y - gyro_offsets[1];
  gyro_readings[2] =  g.gyro.z - gyro_offsets[2];

  accel_readings[0] = a.acceleration.x - accel_offsets[0];
  accel_readings[1] = a.acceleration.y - accel_offsets[1];
  accel_readings[2] = a.acceleration.z - accel_offsets[2];

  // String accel_report = "Accel X: " + String(a.acceleration.x - accel_offsets[0]) + ", Accel Y: " + String(a.acceleration.y - accel_offsets[1]) + ", Accel Z: " + String(a.acceleration.z - accel_offsets[2]) + " m/s^2";

  // String gyro_report = "Rotation X: " + String(g.gyro.x - gyro_offsets[0]) + ", Rotation Y: " + String(g.gyro.y - gyro_offsets[1]) + ", Rotation Z: " + String(g.gyro.z - gyro_offsets[2]) + " rad/s";
  // OLED::display_text(accel_report);
  // delay(100);
}


void IMU::calculate_quantities() {
  read_imu();

  double dt = (millis() - last_imu_time) / 1000;

  // angle += gyro_readings[2] * dt - gyro_z_drift * pow(dt, 2.0);
  angle += gyro_readings[2] * dt - angle_drift * dt;

  if (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  if (angle < -1 * M_PI) {
    angle += 2 * M_PI;
  }

  velocity += accel_readings[0] * dt - velocity_drift * dt - accel_x_drift * pow(dt, 2.0);

  last_imu_time = millis();

  z_accel_history.push_back(accel_readings[2]);

  if (z_accel_history.size() > NUM_Z_ACCEL_HISTORY) {
    z_accel_history.pop_front();
  }

  // String imu_text = "Angle: " + String(angle) + ", Speed: " + String(velocity);
  // OLED::display_text(imu_text);

} 

void IMU::reading_calibrate() {
  double coord_sums[3] = {0};
  double accel_sums[3] = {0};

  for(int i = 0; i < IMU_FAST_CALIBRATION_RUNS; i++){
      read_imu();
      for(int i = 0; i < 3; i++) {
        coord_sums[i] += gyro_readings[i];
        accel_sums[i] += accel_readings[i];
      }
  }

  for(int i = 0; i < 3; i++) {
    gyro_offsets[i] = coord_sums[i] / IMU_FAST_CALIBRATION_RUNS;
    accel_offsets[i] = accel_sums[i] / IMU_FAST_CALIBRATION_RUNS;
  }

  // OLED::display_text("Gyro X Offset: " + String(gyro_offsets[0]) + ", Gyro Y Offset: " + String(gyro_offsets[1]) + ", Gyro Z Offset: " + String(gyro_offsets[2]) + 
  // "Accel X Offset: " + String(accel_offsets[0]) + ", Accel Y Offset: " + String(accel_offsets[1]) + ", Accel Z Offset: " + String(accel_offsets[2]));
}


void IMU::drift_calibrate() {
  read_imu();
  double initial_z_angle_value = gyro_readings[2];
  double initial_x_accel_value = accel_readings[0];

  delay(IMU_SLOW_CALIBRATION_SECONDS * 1000);
  
  read_imu();
  double final_z_angle_value = gyro_readings[2];
  double final_x_accel_value = accel_readings[0];
  
  gyro_z_drift = (final_z_angle_value - initial_z_angle_value) / IMU_SLOW_CALIBRATION_SECONDS;
  accel_x_drift = (final_x_accel_value - initial_x_accel_value) / IMU_SLOW_CALIBRATION_SECONDS;


  // double sum_times = 0;
  // double sum_z_angles = 0;
  // double sum_times_squared = 0;
  // double sum_times_z_angles = 0;
  // double sum_x_accels = 0;
  // double sum_times_x_accels = 0;

  // double num_runs = 0;
  // double start_time = millis();

  // while (millis() - start_time < 1000 * IMU_SLOW_CALIBRATION_SECONDS) {
  //   read_imu();
  //   double time = millis();
  //   sum_times += time;
  //   sum_z_angles += gyro_readings[2];
  //   sum_x_accels += accel_readings[0];
  //   sum_times_squared += pow(time, 2.0);
  //   sum_times_z_angles += time * gyro_readings[2];
  //   sum_times_x_accels += time * accel_readings[0];
  //   num_runs++;
  // }

  // gyro_z_drift = (num_runs * sum_times_z_angles - sum_times * sum_z_angles) / (num_runs * sum_times_squared - pow(sum_times, 2.0));
  // accel_x_drift = (num_runs * sum_times_x_accels - sum_times * sum_x_accels) / (num_runs * sum_times_squared - pow(sum_times, 2.0));

  OLED::display_text("Gyro Z Drift: " + String(gyro_z_drift, 14) + ", Accel X Drift: " + String(accel_x_drift, 14));

}

void IMU::quick_calibration() {
  for (int i = 0; i < 3; i++) {
    gyro_offsets[i] = SAVED_GYRO_OFFSETS[i];
    accel_offsets[i] = SAVED_ACCEL_OFFSETS[i];
  }
  gyro_z_drift = SAVED_Z_ANGLE_DRIFT;
  accel_x_drift = SAVED_X_ACCEL_DRIFT;
  velocity_drift = SAVED_VELOCITY_DRIFT;
}

void IMU::reset_quantities() {
  angle = 0;
  velocity = 0;
  last_imu_time = millis();
}

void IMU::angle_linear_correction() {
  // -0.000001711 // -0.00000247 // -0.000000165
  
  // double sum_times = 0;
  // double sum_z_angles = 0;
  // double sum_times_squared = 0;
  // double sum_times_z_angles = 0;

  // double start_time = millis();
  // double num_runs = 0;
  // while (millis() - start_time < 1000 * IMU_SLOW_CALIBRATION_SECONDS) {
  //   calculate_quantities();
  //   double time = millis();
  //   sum_times += time;
  //   sum_z_angles += angle;
  //   sum_times_squared += pow(time, 2.0);
  //   sum_times_z_angles += time * angle;
  //   num_runs++;
  // }

  // angle_drift = (num_runs * sum_times_z_angles - sum_times * sum_z_angles) / (IMU_VELOCITY_FIT_RUNS * sum_times_squared - pow(sum_times, 2.0));
  double start_time = millis();
  calculate_quantities();
  double start_angle = angle;

  delay(1000 * IMU_SLOW_CALIBRATION_SECONDS);

  double end_time = millis();
  calculate_quantities();
  double end_angle = angle;

  angle_drift = (end_angle - start_angle) / (end_time - start_time);
  
  OLED::display_text("Angle Drift: " + String(angle_drift, 14));
}

// https://www.youtube.com/watch?v=SCZBaOkHB_Y (subtracting linear drift)
// https://www.youtube.com/watch?v=P8hT5nDai6A (how to do linear least squares fit)
void IMU::velocity_linear_correction() {

  double sum_times = 0;
  double sum_velocity = 0;
  double sum_times_squared = 0;
  double sum_times_velocity = 0;

  for (int i = 0; i < IMU_VELOCITY_FIT_RUNS; i++) {
    calculate_quantities();
    double time = millis();
    sum_times += time;
    sum_velocity += velocity;
    sum_times_squared += pow(time, 2.0);
    sum_times_velocity += time * velocity;
  }

  velocity_drift = (IMU_VELOCITY_FIT_RUNS * sum_times_velocity - sum_times * sum_velocity) / (IMU_VELOCITY_FIT_RUNS * sum_times_squared - pow(sum_times, 2.0));
  // OLED::display_text("Velocity Drift: " + String(velocity_drift));
}

IMU::GyroMovement::GyroMovement(IMU &parent_imu) {
  imu = &parent_imu;
}


void IMU::GyroMovement::gyro_turn_absolute(double absolute_angle, double servo_steering_angle, double duty_cycle_offset) {

    if (completed) {return;}

    // // return if you're spamming PWM too fast
    // if (millis() - last_motor_time < 1000 / SERVO_FREQUENCY_HZ) {
    //   return;
    // }

    double angle_difference = (*imu).circular_correction(absolute_angle - (*imu).angle);

    if (angle_difference > 0) {
      servo_steering_angle *= -1;
      motors::left_motor_steering_drive(SERVO_MOUNTING_ANGLE + servo_steering_angle, false, duty_cycle_offset);
      motors::right_motor_steering_drive(SERVO_MOUNTING_ANGLE + servo_steering_angle, false, 0);
    } else {
      motors::left_motor_steering_drive(SERVO_MOUNTING_ANGLE + servo_steering_angle, false, 0);
      motors::right_motor_steering_drive(SERVO_MOUNTING_ANGLE + servo_steering_angle, false, duty_cycle_offset);
    }

    // String angle_text = "Angle: " + String((*imu).angle) + " Difference: " + String(angle_difference) + " Servo write: " + String(SERVO_MOUNTING_ANGLE + servo_steering_angle);
    // OLED::display_text(angle_text);

    motors::servo_pwm(SERVO_MOUNTING_ANGLE + servo_steering_angle);

    // last_motor_time = millis();

    if (abs(angle_difference) < ANGLE_TOLERANCE_RADIANS) {
      completed = true;
    }

}


void IMU::GyroMovement::gyro_turn_relative(double turn_angle, double servo_steering_angle, double duty_cycle_offset) {

   double final_absolute_angle = (*imu).circular_correction((*imu).angle + turn_angle);
   gyro_turn_absolute(final_absolute_angle, servo_steering_angle, duty_cycle_offset);

}


void IMU::GyroMovement::gyro_drive_straight_angle(double target_angle, std::function<bool()> stop_condition, double duty_cycle_offset) {

  if (completed) {return;}

  double error = (*imu).circular_correction(target_angle - (*imu).angle);

  (*imu).proportional = error;
  (*imu).derivative = error - (*imu).last_error;

  if (abs((*imu).integral + error) < (*imu).max_integral) {
    (*imu).integral += error;
  } else if ((*imu).integral + error < 0) {
    (*imu).integral = -1 * (*imu).max_integral;
  } else {
    (*imu).integral = (*imu).max_integral;
  }

  (*imu).last_error = error;

  double correction_val = (*imu).Kp * (*imu).proportional + (*imu).Kd * (*imu).derivative + (*imu).Ki * (*imu).integral;

  // String angle_text = "Angle: " + String((*imu).angle) + " Correction: " + String(correction_val) + " Error: " + String(error) + " Servo: " + String(SERVO_MOUNTING_ANGLE - correction_val);
  // OLED::display_text(angle_text);

  double servo_angle = SERVO_MOUNTING_ANGLE - correction_val;
  motors::servo_pwm(servo_angle);

  double dumb_angle = SERVO_MOUNTING_ANGLE - correction_val;
  if (dumb_angle > SERVO_MOUNTING_ANGLE + SERVO_MAX_STEER) {
      dumb_angle = SERVO_MOUNTING_ANGLE + SERVO_MAX_STEER;
  } else if (dumb_angle < SERVO_MOUNTING_ANGLE - SERVO_MAX_STEER) {
      dumb_angle = SERVO_MOUNTING_ANGLE - SERVO_MAX_STEER;
  }
  motors::left_motor_steering_drive(dumb_angle, false, duty_cycle_offset);
  motors::right_motor_steering_drive(dumb_angle, false, duty_cycle_offset);

  if (stop_condition()) {
    completed = true;
  }
  
}

bool IMU::GyroMovement::complete() {
  return completed;
}

bool IMU::bumpy_terrain() {

  double sum = 0;
  for (double value : z_accel_history) {
    sum += abs(value);
  }

  double average = sum / z_accel_history.size();

  OLED::display_text(String(average) + " " + String(average > BUMPY_DEVIATION_THRESHOLD));

  return average > BUMPY_DEVIATION_THRESHOLD;
}

bool IMU::rubble_falling_edge() {
  static bool rubbling = false;
  static bool prev_rubbling = false;

  rubbling = bumpy_terrain();

  if (!rubbling && prev_rubbling) {
    prev_rubbling = rubbling;
    return true;
  }

  prev_rubbling = rubbling;
  return false;
}

bool IMU::rubble_rising_edge() {
  static bool rubbling = false;
  static bool prev_rubbling = false;

  rubbling = bumpy_terrain();

  if (rubbling && !prev_rubbling) {
    prev_rubbling = rubbling;
    return true;
  }

  prev_rubbling = rubbling;
  return false;
}

bool IMU::correct_orientation(double target_angle) {
  return abs(circular_correction(angle - target_angle)) < ANGLE_TOLERANCE_RADIANS;
}

double IMU::circular_correction(double angle) {

  if (angle > M_PI) {
    angle -= 2 * M_PI;
  }

  else if (angle < -1 * M_PI ) {
    angle += 2 * M_PI;
  }

  return angle;
}