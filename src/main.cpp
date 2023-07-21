#include <Arduino.h>
#include <tape_following.h>
#include <global_values.h>
#include <gyro.h>
#include <sonar.h>

void setup() {
  // pins
  pinMode(SERVO_PIN, OUTPUT);
  for (int i = 0; i < NUM_IR_SENSORS; i++)  {
      pinMode(IR_PINS[i], INPUT);
  }

  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(LEFT_REVERSE_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_REVERSE_MOTOR_PIN, OUTPUT);

  // pinMode(SONAR_TRIGGER_PIN_1, OUTPUT);
  // pinMode(SONAR_ECHO_PIN_1, INPUT);

  // // i2c adafruit components
  begin_oled();
  begin_gyro();

  // gyro calibration
  velocity_calibrate();
  display_text("fast calibration complete!");
  delay(2000);

  slow_calibrate();
  display_text("slow calibration complete!");
  delay(2000);

  // ir calibration
  // constant_offset_tape_sensors();
}

void loop() {
  calculate_angle(); // needs to be called in EVERY loop iteration, regardless of stage to keep angle reading correct
  drive_straight_angle_pid(0);
  // left_motor_PWM(30);
  // right_motor_PWM(30);
  // tape_follow_drive();
  // calculate_angle();
  // drive_straight_angle_pid(1);
  // if (millis() - last_time > 5000) {
  //   gyro_turn_absolute(1, 0.4);
  //   last_time = millis();
  // } else {
  //   display_text("done turn!");
  // }
  // left_motor_PWM(60);
  // servo_pwm(M_PI);
  // delay(5000);
  // left_motor_PWM(-60);
  // servo_pwm(0);
  // delay(5000);
  // right_motor_PWM(60);
  // servo_pwm(M_PI);
  // delay(5000);
  // right_motor_PWM(-60);
  // servo_pwm(0);
  // delay(5000);
  // // display_text("hi");
  // // delay(3);
}


//{
  /*
  STATE 1: (triggered by start)
  pwm start both motors straight (possible gyro PID)
  delay x seconds
  gyro absolute turn to 0 degrees

  STATE 2: (triggered after STATE 1)
  gyro PID straight at 0 degrees

  STATE 3: (triggered by bridge sonar)
  some delay??
  gyro PID straight towards wall, some angle

  STATE 4: (triggered by wall sonar)
  gyro absolute turn to 0 degrees
  possibly slow down to not go off cliff, guess delay?
  possibly start turning early, based on delay

  STATE 5: (triggered by tape after wall)
  possibly back off after seeing tape
  gyro absolute turn to 180 degrees, tight turn
  gyro PID straight at 180 degrees (possibly tape follow)

  STATE 6: (triggered by big accelerometer spike)
  gyro absolute turn to 90 degrees, more wide because rock
  
  STATE 7: (triggered by IR seeing white)
  gyro absolute turn to 0 degrees, tight turn
  back to STATE 3

  */
//}
