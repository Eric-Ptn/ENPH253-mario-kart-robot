#include <mario-kart-robot\imu.h>
#include <mario-kart-robot\oled_display.h>
#include <mario-kart-robot\motors.h>
#include <cmath>

void IMU::begin_imu() {
    imu.begin();
}


void IMU::read_imu() {
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  gyro_readings[0] = g.gyro.x - gyro_offsets[0];
  gyro_readings[1] = g.gyro.y - gyro_offsets[1];
  gyro_readings[2] =  g.gyro.z - gyro_offsets[2];

  accel_readings[0] = a.acceleration.x - accel_offsets[0];
  accel_readings[1] = a.acceleration.y - accel_offsets[1];
  accel_readings[2] = a.acceleration.z - accel_offsets[2];

  // String gyro_report = "Rotation X: " + String(g.gyro.x - gyro_offsets[0]) + ", Rotation Y: " + String(g.gyro.y - gyro_offsets[1]) + ", Rotation Z: " + String(g.gyro.z - gyro_offsets[2]) + " rad/s";
  // OLED::display_text(gyro_report);
}


double IMU::calculate_z_angle() {
  read_imu();

  double dt = (millis() - angle_time) / 1000;
  angle += gyro_readings[2] * dt - gyro_z_drift * pow(dt, 2.0);
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

IMU::GyroMovement::GyroMovement(IMU &parent_imu) {
  imu = &parent_imu;
}

void IMU::reset_angle() {
  angle = 0;
  angle_time = millis();
}

void IMU::GyroMovement::gyro_turn_absolute(double absolute_angle, double servo_steering_angle) {

    if (completed) {return;}

    double angle_difference = (*imu).circular_correction(absolute_angle - (*imu).angle);

    if (angle_difference > 0) {
      servo_steering_angle *= -1;
    }

    String angle_text = "Angle: " + String((*imu).angle) + " Difference: " + String(angle_difference);
    OLED::display_text(angle_text);

    motors::servo_pwm(SERVO_MOUNTING_ANGLE + servo_steering_angle);
    motors::left_motor_steering_drive(SERVO_MOUNTING_ANGLE + servo_steering_angle, false, false); // TODO: change later
    motors::right_motor_steering_drive(SERVO_MOUNTING_ANGLE + servo_steering_angle, false, false);

    if (abs(angle_difference) < ANGLE_TOLERANCE_RADIANS) {
      completed = true;
      String text = "done";
      OLED::display_text(text);
    }

}


void IMU::GyroMovement::gyro_turn_relative(double turn_angle, double servo_steering_angle) {

   double final_absolute_angle = (*imu).circular_correction((*imu).angle + turn_angle);
   gyro_turn_absolute(final_absolute_angle, servo_steering_angle);

}


void IMU::GyroMovement::gyro_drive_straight_angle(double target_angle, std::function<bool()> stop_condition) {

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

  motors::servo_pwm(SERVO_MOUNTING_ANGLE + correction_val);
  motors::left_motor_steering_drive(SERVO_MOUNTING_ANGLE + correction_val, false, false); // change later
  motors::right_motor_steering_drive(SERVO_MOUNTING_ANGLE + correction_val, false, false);

  if (stop_condition()) {
    completed = true;
  }
  
}

bool IMU::GyroMovement::complete() {
  return completed;
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