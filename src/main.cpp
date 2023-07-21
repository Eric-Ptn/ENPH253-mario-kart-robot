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

  pinMode(BRIDGE_SONAR_TRIGGER, OUTPUT);
  pinMode(BRIDGE_SONAR_ECHO, INPUT);
  pinMode(WALL_SONAR_TRIGGER, OUTPUT);
  pinMode(WALL_SONAR_ECHO, INPUT);

  // gyro and OLED connect to I2C pins, PB6 and PB7, but don't need to be included here


  // i2c adafruit components
  begin_oled();
  begin_gyro();

  // ir calibration
  constant_offset_tape_sensors();

  // gyro calibration
  velocity_calibrate();
  display_text("fast calibration complete!");
  delay(1000);
  display_text("slow calibration...");
  slow_calibrate();
}

// TEST GYRO STRAIGHT PID

void loop() {
  calculate_angle();
  drive_straight_angle_pid(0);
}

// TEST GYRO TURNING

// double last_time = millis();
// void loop() {
//   calculate_angle();
//   drive_straight_angle_pid(1);
//   if (millis() - last_time > 5000) {
//     gyro_turn_absolute(1, 0.4);
//     last_time = millis();
//   } else {
//     display_text("done turn!");
//   }
// }

// TEST TAPE FOLLOWING

// void loop() {
//   tape_follow_drive();
// }



// HIGH LEVEL PSEUDOCODE

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

// FULL STATE MACHINE PROGRAM ???

enum ROBOT_STATES {
  START,
  GUN_FOR_BRIDGE_FROM_START,
  GUN_FOR_WALL_FROM_BRIDGE,
  WALL_RIDE,
  UP_RAMP,
  TURN_AFTER_FALL,
  TURN_TO_FACE_BRIDGE
};

enum ROBOT_STATES current_state = START;

void loop() {
  calculate_angle();  // needs to be called in EVERY loop iteration, regardless of stage to keep gyro angle reading correct

  switch (current_state) {
    case START:
      // alternatively, may tape follow until the first sharp turn (gyro reads 0), then switch to gun bridge
      left_motor_PWM(DEFAULT_MOTOR_DUTY_CYCLE);
      right_motor_PWM(DEFAULT_MOTOR_DUTY_CYCLE);
      delay(500);
      current_state = GUN_FOR_BRIDGE_FROM_START;
      break;

    case GUN_FOR_BRIDGE_FROM_START:
      gyro_turn_absolute(0, 0.8);
      drive_straight_angle_pid(0);

      trigger_sonar(BRIDGE_SONAR_TRIGGER);
      if (measure_distance(BRIDGE_SONAR_ECHO) < BRIDGE_DISTANCE) {
        current_state = GUN_FOR_WALL_FROM_BRIDGE;
      }
      break;

    case GUN_FOR_WALL_FROM_BRIDGE:
      gyro_turn_absolute(-1, 0.8);
      drive_straight_angle_pid(-1);
      trigger_sonar(WALL_SONAR_TRIGGER);
      if (measure_distance(WALL_SONAR_ECHO) < WALL_DISTANCE) {
        current_state = WALL_RIDE;
      }
      break;

    case WALL_RIDE:
      gyro_turn_absolute(0, 0.8);
      drive_straight_angle_pid(0);
      if (blabla ir see black) {
        current_state = UP_RAMP;
      }
      break;
  }
}
