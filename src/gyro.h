#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <oled_display.h>
#include <math.h>
#include <global_values.h>

Adafruit_MPU6050 gyro;

// static variables for local file scope
static double gyro_readings[3];
static double gyro_offsets[] = {0, 0, 0}; 
static double angle = 0;
static double angle_time = 0; // start angle time on first run?
static double z_drift = 0;

// PID vals
static double derivative = 0;
static double integral = 0;
static double proportional = 0;
static double last_error = 0;
// PID gains 
static int Kd_gyro = 0;
static int Ki_gyro = 0;
static int Kp_gyro = 1;


void begin_gyro() {
    gyro.begin();
}

void read_gyro() {
  sensors_event_t a, g, temp;
  gyro.getEvent(&a, &g, &temp);

  gyro_readings[0] = g.gyro.x - gyro_offsets[0];
  gyro_readings[1] = g.gyro.y - gyro_offsets[1];
  gyro_readings[2] =  g.gyro.z - gyro_offsets[2] - z_drift;

  // String gyro_report = "Rotation X: " + String(g.gyro.x - gyro_offsets[0]) + ", Rotation Y: " + String(g.gyro.y - gyro_offsets[1]) + ", Rotation Z: " + String(g.gyro.z - gyro_offsets[2]) + " rad/s";
  // display_text(gyro_report);
}

/**
Purpose: Measures the value of the x,y,z velocities when the gyro is stationary. Used for calibration.
Alters: Saves the value of the velocity
NOTE*: THE GYRO MUST BE STATIONARY WHEN CALIBRATING.
*/
void velocity_calibrate() {
  double coord_sums[] = {0, 0, 0};

  for(int i = 0; i < GYRO_FAST_CALIBRATION_RUNS; i++){
      read_gyro();
      for(int i = 0; i < 3; i++) {
        coord_sums[i] += gyro_readings[i];
      }
  }

  for(int i = 0; i < 3; i++) {
    gyro_offsets[i] = coord_sums[i] / GYRO_FAST_CALIBRATION_RUNS;
  }
}

/*
Purpose: Measures the drift of the z-angle of the gyro. If this drift is assumed to be linear, it can be subtracted from subsequent
         angle measurements to reduce drift.
Alters: 
  -z-drift (should only be altered by this function): uses this variable to record the drift.
*/
void slow_calibrate() {
  read_gyro();

  double initial_value = gyro_readings[2];
  delay(GYRO_SLOW_CALIBRATION_SECONDS * 1000);
  
  read_gyro();
  double final_value = gyro_readings[2];
  
  z_drift = (final_value - initial_value) / GYRO_SLOW_CALIBRATION_SECONDS;
}

/*
Purpose: Calculates the angle that the gyro is oriented at. This angle is in the range -pi to pi.
Alters: 
  - angle:  angle of the gyro, starts at 0.
*/
double calculate_angle() {
  read_gyro();

  double dt = (millis() - angle_time) / 1000;
  angle += gyro_readings[2] * dt;
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  if (angle < -1 * M_PI) {
    angle += 2 * M_PI;
  }

  angle_time = millis();

  String angle_text = "Angle: " + String(angle);
  display_text(angle_text);
  return angle;
}

/*
Purpose: Turns to a specified angle.
Params:
  - absolute_angle: specifies the angle that the vehicle is to be turned to.
  - servo_steering_angle: specifies the angle that the servo will steer the vehicle at (specifies the turning radius)
*/
void gyro_turn_absolute(double absolute_angle, double servo_steering_angle) {
    servo_pwm(SERVO_MOUNTING_ANGLE + servo_steering_angle);
    left_motor_PWM(DEFAULT_MOTOR_DUTY_CYCLE);
    right_motor_PWM(DEFAULT_MOTOR_DUTY_CYCLE);

    while(abs(gyro_readings[2] - absolute_angle) > ANGLE_TOLERANCE_RADIANS) read_gyro();
}

/*
Purpose: Drives the vehicle straight, at a given angle (this version uses proportional control).
Params:
  - absolute_angle: the angle that the vehicle is driving at.
*/
void drive_straight_angle(double absolute_angle) {
    read_gyro();
    // scale steering angle based on deviation from desired angle, proportional control
    double servo_adjustment_angle = min(gyro_readings[2] - absolute_angle, min(gyro_readings[2] - absolute_angle + 2 * M_PI, gyro_readings[2] - absolute_angle - 2 * M_PI));

    gyro_turn_absolute(absolute_angle, servo_adjustment_angle); 
}

/*
Purpose: Turns vehicle BY a specified angle (the nuance is in the wording).
Params:
  - turning_angle: specifies the angle that the vehicle is to be turned BY.
  - servo_steering_angle: specifies the angle that the servo will steer the vehicle at (specifies the turning radius)
*/
void gyro_turn_relative(double turn_angle, double servo_steering_angle) {

   read_gyro();
   double final_absolute_angle = gyro_readings[2] + turn_angle;

   if (final_absolute_angle > M_PI) {
    final_absolute_angle -= 2 * M_PI;
   }
   if (final_absolute_angle < -1 * M_PI) {
    final_absolute_angle += 2 * M_PI;
   }

   gyro_turn_absolute(final_absolute_angle, servo_steering_angle);
}

/*
Purpose: Uses PID control and the gyro to drive along a given angle. 
Params: 
  - target_angle: the angle that we want to drive the vehicle at.
*/
void drive_straight_angle_pid (double target_angle) {

  read_gyro();

  double error = gyro_readings[2] - target_angle;

  if (error > M_PI) {
    error -= 2 * M_PI;
  }

  else if (error < -1 * M_PI ) {
    error += 2 * M_PI;
  }


  proportional = error;
  derivative = error - last_error;
  integral += error; 

  double correction_val = Kp_gyro * proportional + Kd_gyro * derivative + Ki_gyro * integral;

  servo_pwm(SERVO_MOUNTING_ANGLE - correction_val);

  
}
