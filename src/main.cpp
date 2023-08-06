#include <Arduino.h>
#include <mario-kart-robot\config.h>
#include <mario-kart-robot\motors.h>
#include <mario-kart-robot\imu.h>
#include <mario-kart-robot\tape_follower.h>
#include <mario-kart-robot\oled_display.h>
#include <mario-kart-robot\sonar.h>

IMU mpu6050;
TapeFollower tape_follower;

const int num_straights = 4;
IMU::GyroMovement* straight_moves[num_straights];

const int num_turns = 6;
IMU::GyroMovement* turn_moves[num_turns];

double last_blink_time = 0;

void blink_dat() {
  if (millis() - last_blink_time < 500) {
    return;
  }
  if (digitalRead(PC13)) {
    digitalWrite(PC13, LOW);
    last_blink_time = millis();
  } else {
    digitalWrite(PC13, HIGH);
    last_blink_time = millis();
  }
}

void reset_gyro_move_arrays() {
  for (int i = 0; i < num_straights; i++) {
    straight_moves[i] = new IMU::GyroMovement(mpu6050);
  }
  for (int i = 0; i < num_turns; i++) {
    turn_moves[i] = new IMU::GyroMovement(mpu6050);
  }
}

void setup() {
  // Serial.begin(115200);
  // Serial.print("started");
  // CHANGE WIRE OBJECT TO WORK ON SECOND I2C
  Wire.begin(uint32_t(PB11), uint32_t(PB10));
  // Serial.print("i2c begin \n");

  // Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */); // unfortunately i don't think stm32 Wire has this function
  
  // i2c adafruit components
  OLED::begin_oled();
  mpu6050.begin_imu();
  // Serial.print("i2c begin components \n");


  OLED::display_text("setting up...");

  // // initialize gyro movement objects
  reset_gyro_move_arrays();
  // Serial.print("initialize gyro arrays \n");


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
  // Serial.print("set pin modes \n");


  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(PC13, OUTPUT);

  // // gyro calibration
  delay(1000);
  OLED::display_text("gyro fast calibration...");
  mpu6050.reading_calibrate();
  OLED::display_text("gyro slow calibration...");
  mpu6050.drift_calibrate();
  mpu6050.velocity_linear_correction();
  mpu6050.reset_quantities();

  // ir calibration
  delay(100);
  OLED::display_text("tape calibration...");
  // tape_follower.tape_calibration();
  tape_follower.scaling_offset_calibration();
  // Serial.print("calibrated \n");


  // OLED::display_text("done calibration!");


  while(1) {
    if (digitalRead(START_BUTTON) == LOW) {
        break;
    }
    mpu6050.calculate_quantities();
    OLED::display_text("s0: " + String(tape_follower.processed_ir_reading(0)) + ", " + String(tape_follower.ir_reading_no_threshold(0)) +
                      " s1: " + String(tape_follower.processed_ir_reading(1)) + ", " + String(tape_follower.ir_reading_no_threshold(1)) +
                      " s2: " + String(tape_follower.processed_ir_reading(2)) + ", " + String(tape_follower.ir_reading_no_threshold(2)) +
                      " s3: " + String(tape_follower.processed_ir_reading(3)) + ", " + String(tape_follower.ir_reading_no_threshold(3)));
  }

 
 motors::servo_pwm(SERVO_MOUNTING_ANGLE);

//  Serial.print("moved servo \n");

}

// bool left = true;
// double servo_angle = SERVO_MOUNTING_ANGLE;

// TEST NOISE ISSUES - SERVO SWEEP
// void loop () {
//   // mpu6050.calculate_quantities();
//   // // OLED::display_text(String(millis()));
//   // motors::left_motor_PWM(30);
//   // motors::right_motor_PWM(30);
//   // if (digitalRead(PC13) == HIGH) {
//   //   digitalWrite(PC13, LOW);
//   // } else {
//   //   digitalWrite(PC13, HIGH);
//   // }
//   // motors::right_motor_PWM(30);
//   if (left) {
//     servo_angle += 0.01;
//     if (servo_angle > SERVO_MOUNTING_ANGLE + SERVO_MAX_STEER) {
//       left = false;
//     }
//   } else {
//     servo_angle -= 0.01;
//     if (servo_angle < SERVO_MOUNTING_ANGLE - SERVO_MAX_STEER) {
//       left = true;
//     }
//   }
//   motors::servo_pwm(servo_angle);  delay(5);
// }

// TEST GYRO STRAIGHT PID

// IMU::GyroMovement straight1(mpu6050);
// // auto test_bool_ptr = std::bind(&TapeFollower::test_bool, tape_follower);
// auto sonar_ptr = std::bind(&sonar::test_bool); // smth like this

// void loop() {
//   // motors::servo_pwm(SERVO_MOUNTING_ANGLE);
//   mpu6050.calculate_quantities();
//   straight1.gyro_drive_straight_angle(0, sonar_ptr);
// }

// TEST GYRO TURN

// IMU::GyroMovement turn1(mpu6050);

// void loop() {
//   mpu6050.calculate_quantities();  
//   turn1.gyro_turn_absolute(M_PI / 2, 0.3);
//   if (turn1.complete()) {
//     OLED::display_text("turn done!!");
//     motors::left_motor_PWM(0);
//     motors::right_motor_PWM(0);
//   }
// }

// TEST TAPE FOLLOWING PID
// servo write:1.55, correction val:0, position -1

// void loop() {
//   // motors::servo_pwm(SERVO_MOUNTING_ANGLE);
//   tape_follower.follow_tape();
  
//   blink_dat();
// }

// SONAR DETECTION

// void loop() {
//   sonar::trigger_sonar(BRIDGE_SONAR_TRIGGER);
//   OLED::display_text(String(sonar::measure_distance(BRIDGE_SONAR_ECHO)));
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

// FULL STATE MACHINE LOOP

enum ROBOT_STATES {
  RIGHT_START,
  LEFT_START,
  GUN_FOR_BRIDGE_FROM_START,
  GUN_FOR_WALL_FROM_BRIDGE,
  WALL_RIDE,
  UP_RAMP,
  TURN_AFTER_FALL
};

enum ROBOT_STATES current_state = LEFT_START;

// binding pointers to functions
auto test_ptr = std::bind(&sonar::test_bool);
auto bridge_ptr = std::bind(&sonar::seeing_bridge_falling_edge);
auto wall_ptr = std::bind(&sonar::seeing_wall);
auto black_tape_ptr = std::bind(&TapeFollower::seeing_black, tape_follower);
auto falling_ptr = std::bind(&IMU::robot_falling, mpu6050);
auto white_bg_ptr = std::bind(&TapeFollower::seeing_white, tape_follower);


void loop() {
  mpu6050.calculate_quantities(); // MUST BE CALLED EVERY LOOP

  switch (current_state) {
    // case START:
    //   // tape follow until the first sharp turn (gyro reads 0), then switch to gun bridge
    //   // alternatively may hardcode first turn
    //   tape_follower.follow_tape();
    //   if (mpu6050.correct_orientation(0)) {  
    //     current_state = GUN_FOR_BRIDGE_FROM_START;
    //   }
    //   break;
    case RIGHT_START:
      (*turn_moves[0]).gyro_turn_absolute(0.35, SERVO_MAX_STEER / 2, 5);
      if ((*turn_moves[0]).complete()) {
        (*straight_moves[0]).gyro_drive_straight_angle(0.35, test_ptr);
      }
      break;

    case LEFT_START:
      (*turn_moves[0]).gyro_turn_absolute(-0.15, SERVO_MAX_STEER / 2, 5); // interestingly this works, can turn to slightly off angle and then straight PID back to 0
      if ((*turn_moves[0]).complete()) {
        (*straight_moves[0]).gyro_drive_straight_angle(0, test_ptr);
      }
      break;

    case GUN_FOR_BRIDGE_FROM_START:
      sonar::trigger_sonar(BRIDGE_SONAR_TRIGGER);

      (*straight_moves[0]).gyro_drive_straight_angle(0, test_ptr);

      if ((*straight_moves[0]).complete()) {
        current_state = GUN_FOR_WALL_FROM_BRIDGE;
        sonar::stop_sonar(BRIDGE_SONAR_TRIGGER);
      }
      break;

    case GUN_FOR_WALL_FROM_BRIDGE:
    // this will be different depending on starting location of robot
      (*turn_moves[0]).gyro_turn_absolute(-1, SERVO_MAX_STEER);

      if ((*turn_moves[0]).complete()) {
        sonar::trigger_sonar(WALL_SONAR_TRIGGER);
        (*straight_moves[1]).gyro_drive_straight_angle(-1, wall_ptr);
      }

      if ((*straight_moves[1]).complete()) {
        current_state = WALL_RIDE;
        sonar::stop_sonar(WALL_SONAR_TRIGGER);
      }
      break;

    case WALL_RIDE:
      (*turn_moves[1]).gyro_turn_absolute(0, SERVO_MAX_STEER);

      if ((*turn_moves[1]).complete()) {
        (*straight_moves[1]).gyro_drive_straight_angle(0, black_tape_ptr);
      }

      if ((*straight_moves[1]).complete()) {
        current_state = UP_RAMP;
      }
      break;
    
    case UP_RAMP:
      // alternatively, tape follow up the ramp and go straight at signal, then may hardcode only 90 deg turn

      (*turn_moves[2]).gyro_turn_absolute(M_PI / 2, SERVO_MAX_STEER);
      if ((*turn_moves[2]).complete()) {
        (*turn_moves[3]).gyro_turn_absolute(M_PI, SERVO_MAX_STEER);
      }
      if ((*turn_moves[3]).complete()) {
        (*straight_moves[2]).gyro_drive_straight_angle(M_PI, falling_ptr, 5);
      }
      if ((*straight_moves[2]).complete()) {
        current_state = TURN_AFTER_FALL;
      }
      break;

    case TURN_AFTER_FALL:
      (*turn_moves[4]).gyro_turn_absolute(-1 * M_PI / 2, SERVO_MAX_STEER / 2, 5);

      if (tape_follower.seeing_white() && (*turn_moves[4]).complete()) {

        (*turn_moves[5]).gyro_turn_absolute(0, SERVO_MAX_STEER);

      } else if ((*turn_moves[4]).complete()) {

        // keep moving forward if you're done the 90 deg turn but haven't seen white yet
        (*straight_moves[3]).gyro_drive_straight_angle(-1 * M_PI / 2, white_bg_ptr, 5);

      }

      if ((*turn_moves[5]).complete()) {
        current_state = GUN_FOR_BRIDGE_FROM_START;
        reset_gyro_move_arrays();
      }
      break;

    default:
      OLED::display_text("defaulted");
      motors::servo_pwm(SERVO_MOUNTING_ANGLE);
      motors::left_motor_PWM(0);
      motors::right_motor_PWM(0);

 }
}
