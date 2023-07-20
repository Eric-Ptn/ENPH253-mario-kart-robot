#include <Arduino.h>
#include <global_values.h>
#include <math.h>

// https://components101.com/motors/mg996r-servo-motor-datasheet

/*
Purpose: Moves the servo to a specified angle.
Params:
  - angle: angle that the servo is to be moved to.
*/
void servo_pwm(double angle) {

  // ~0 radians is 2% duty cycle, ~pi radians is 12.5% duty cycle
  // servo.h does NOT work nicely, use this instead

  double duty_cycle_percent = (12.5 - 2.000000000)/(M_PI - 0.0) * (angle - 0.0) + 2.0;
  // double duty_cycle_percent = (9.0 - 2.0)/( 2 * M_PI / 3 - 0.0) * (angle - 0.0) + 2.0;

  pwm_start(SERVO_PIN_PWM_NAME, SERVO_FREQUENCY_HZ, duty_cycle_percent / 100 * 65536, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
}

/*
Purpose: Following functions are used to drive the left and right motors respectively.
Params:
  - duty_cycle_percent: duty cycle of the pwm to the motor (determines motor speed)
*/
void left_motor_PWM(double duty_cycle_percent) {
  if (duty_cycle_percent > 0) {
    pwm_start(LEFT_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, duty_cycle_percent, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    pwm_start(LEFT_REVERSE_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, 0, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
  } else {
    pwm_start(LEFT_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, 0, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    pwm_start(LEFT_REVERSE_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, -duty_cycle_percent, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
  }
}

void right_motor_PWM(double duty_cycle_percent) {
  if (duty_cycle_percent > 0) {
    pwm_start(RIGHT_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, duty_cycle_percent, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    pwm_start(RIGHT_REVERSE_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, 0, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
  } else {
    pwm_start(RIGHT_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, 0, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    pwm_start(RIGHT_REVERSE_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, -duty_cycle_percent, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
  }
}
