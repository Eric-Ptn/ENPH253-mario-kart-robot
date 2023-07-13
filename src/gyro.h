#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <oled_display.h>
#include <math.h>

Adafruit_MPU6050 gyro;

// static variables for local file scope
static double gyro_readings[3];
static double gyro_offsets[] = {0, 0, 0}; 
static double angle = 0;
static double angle_time = 0; // start angle time on first run?
static double z_drift = 0;

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

void velocity_calibrate() {
  double coord_sums[] = {0, 0, 0};

  for(int i = 0; i < FAST_CALIBRATION_RUNS; i++){
      read_gyro();
      for(int i = 0; i < 3; i++) {
        coord_sums[i] += gyro_readings[i];
      }
  }

  for(int i = 0; i < 3; i++) {
    gyro_offsets[i] = coord_sums[i] / FAST_CALIBRATION_RUNS;
  }
}

void slow_calibrate() {
  read_gyro();

  double initial_value = gyro_readings[2];
  delay(SLOW_CALIBRATION_SECONDS * 1000);
  
  read_gyro();
  double final_value = gyro_readings[2];
  
  z_drift = (final_value - initial_value) / SLOW_CALIBRATION_SECONDS;
}

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

void drive_straight_angle() {
   
}

void gyro_turn(double turn_angle, double servo_steering_angle) {
   
}