#include <mario-kart-robot\imu.h>
#include <mario-kart-robot\oled_display.h>
#include <mario-kart-robot\motors.h>


void IMU::begin_imu() {
    imu.begin();
}


void IMU::read_imu() {
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  gyro_readings[0] = g.gyro.x - gyro_offsets[0];
  gyro_readings[1] = g.gyro.y - gyro_offsets[1];
  gyro_readings[2] =  g.gyro.z - gyro_offsets[2] - gyro_z_drift;

  accel_readings[0] = a.acceleration.x - accel_readings[0];
  accel_readings[1] = a.acceleration.y - accel_readings[1];
  accel_readings[2] = a.acceleration.z - accel_readings[2];

  // String gyro_report = "Rotation X: " + String(g.gyro.x - gyro_offsets[0]) + ", Rotation Y: " + String(g.gyro.y - gyro_offsets[1]) + ", Rotation Z: " + String(g.gyro.z - gyro_offsets[2]) + " rad/s";
  // OLED::display_text(gyro_report);
}


double IMU::calculate_z_angle() {
  read_imu();

  double dt = (millis() - angle_time) / 1000;
  angle += gyro_readings[2] * dt;
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  if (angle < -1 * M_PI) {
    angle += 2 * M_PI;
  }

  angle_time = millis();

  // String angle_text = "Angle: " + String(angle);
  // OLED::display_text(angle_text);
  return angle;
}


void IMU::reading_calibrate() {
  double coord_sums[3] = {0};
  double accel_sums[3] = {0};

  for(int i = 0; i < GYRO_FAST_CALIBRATION_RUNS; i++){
      read_imu();
      for(int i = 0; i < 3; i++) {
        coord_sums[i] += gyro_readings[i];
        accel_sums[i] += accel_readings[i];
      }
  }

  for(int i = 0; i < 3; i++) {
    gyro_offsets[i] = coord_sums[i] / GYRO_FAST_CALIBRATION_RUNS;
    accel_offsets[i] = accel_sums[i] / GYRO_FAST_CALIBRATION_RUNS;
  }
}


void IMU::z_drift_calibrate() {
  read_imu();
  double initial_value = gyro_readings[2];
  delay(GYRO_SLOW_CALIBRATION_SECONDS * 1000);
  
  read_imu();
  double final_value = gyro_readings[2];
  
  gyro_z_drift = (final_value - initial_value) / GYRO_SLOW_CALIBRATION_SECONDS;
}


void IMU::gyro_turn_absolute(double absolute_angle, double servo_steering_angle) {

    // need to steer servo to correct direction
    double angle_difference = circular_correction(absolute_angle - angle);

    if (angle_difference < 0) {
      servo_steering_angle *= -1;
    }
    motors::servo_pwm(SERVO_MOUNTING_ANGLE + servo_steering_angle);
    motors::left_motor_steering_drive(SERVO_MOUNTING_ANGLE + servo_steering_angle, false);
    motors::right_motor_steering_drive(SERVO_MOUNTING_ANGLE + servo_steering_angle, false);

    while (abs(angle_difference) > ANGLE_TOLERANCE_RADIANS) {
      calculate_z_angle();
      angle_difference = circular_correction(absolute_angle - angle);
      // String angle_text = "Angle: " + String(angle) + " Difference: " + String(angle_difference);
      // OLED::display_text(angle_text);
    }
}


void IMU::gyro_turn_relative(double turn_angle, double servo_steering_angle) {

   double final_absolute_angle = circular_correction(angle + turn_angle);
   gyro_turn_absolute(final_absolute_angle, servo_steering_angle);

}


void IMU::gyro_drive_straight_angle(double target_angle, bool (*stop_condition)()) {

  while (!stop_condition()) {
    double error = circular_correction(target_angle - angle);

    proportional = error;
    derivative = error - last_error;

    if (abs(integral + error) < max_integral) {
      integral += error;
    } else if (integral + error < 0) {
      integral = -1 * max_integral;
    } else {
      integral = max_integral;
    }

    last_error = error;

    double correction_val = Kp * proportional + Kd * derivative + Ki * integral;

    // String angle_text = "Angle: " + String(angle) + " Correction: " + String(correction_val) + " Error: " + String(error);
    // OLED::display_text(angle_text);

    motors::servo_pwm(SERVO_MOUNTING_ANGLE + correction_val);
    motors::left_motor_steering_drive(SERVO_MOUNTING_ANGLE + correction_val, false);
    motors::right_motor_steering_drive(SERVO_MOUNTING_ANGLE + correction_val, false);
  }
  
}

bool IMU::robot_falling() {
  read_imu();
  return accel_readings[2] > FALLING_ACCELERATION;
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