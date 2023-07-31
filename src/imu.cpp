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

  angle += gyro_readings[2] * dt - gyro_z_drift * pow(dt, 2.0);
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  if (angle < -1 * M_PI) {
    angle += 2 * M_PI;
  }

  velocity += accel_readings[0] * dt - velocity_drift * dt - accel_x_drift * pow(dt, 2.0);

  last_imu_time = millis();

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

  // OLED::display_text("Gyro Z Drift: " + String(gyro_z_drift) + ", Accel X Drift: " + String(accel_x_drift));
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

    double angle_difference = (*imu).circular_correction(absolute_angle - (*imu).angle);

    if (angle_difference > 0) {
      servo_steering_angle *= -1;
    }

    String angle_text = "Angle: " + String((*imu).angle) + " Difference: " + String(angle_difference);
    OLED::display_text(angle_text);

    motors::servo_pwm(SERVO_MOUNTING_ANGLE + servo_steering_angle);
    motors::left_motor_steering_drive(SERVO_MOUNTING_ANGLE + servo_steering_angle, false, duty_cycle_offset);
    motors::right_motor_steering_drive(SERVO_MOUNTING_ANGLE + servo_steering_angle, false, duty_cycle_offset);

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

  String angle_text = "Angle: " + String((*imu).angle) + " Correction: " + String(correction_val) + " Error: " + String(error);
  OLED::display_text(angle_text);

  motors::servo_pwm(SERVO_MOUNTING_ANGLE - correction_val);
  motors::left_motor_steering_drive(SERVO_MOUNTING_ANGLE + correction_val, false, duty_cycle_offset);
  motors::right_motor_steering_drive(SERVO_MOUNTING_ANGLE + correction_val, false, duty_cycle_offset);

  if (stop_condition()) {
    completed = true;
  }
  
}

bool IMU::GyroMovement::complete() {
  return completed;
}

bool IMU::robot_falling() {
  return accel_readings[2] > FALLING_ACCELERATION;
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