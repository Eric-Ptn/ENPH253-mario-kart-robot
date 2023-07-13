#include <Arduino.h>
#include <global_values.h>

// https://components101.com/motors/mg996r-servo-motor-datasheet
void servo_pwm(double angle) {

  // ~0 degrees is 2% duty cycle, ~180 degrees is 10% duty cycle
  // servo.h does NOT work nicely, use this instead
  double duty_cycle_percent = (10.0 - 3.0)/(180.0 - 0.0) * (angle - 0.0) + 3.0;

  pwm_start(SERVO_PIN_PWM_NAME, SERVO_FREQUENCY_HZ, duty_cycle_percent, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
}

void left_motor_PWM(double duty_cycle_percent) {
    pwm_start(LEFT_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, duty_cycle_percent, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
}

void right_motor_PWM(double duty_cycle_percent) {
    pwm_start(RIGHT_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, duty_cycle_percent, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
}
