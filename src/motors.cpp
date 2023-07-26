#include <Arduino.h>
#include <math.h>
#include <mario-kart-robot\motors.h>
#include <mario-kart-robot\config.h>

namespace motors {

  void servo_pwm(double angle) {

    // ~0 radians is 2% duty cycle, ~pi radians is 12.5% duty cycle
    // servo.h does NOT work nicely, use this instead

    double limited_angle = angle;
    if (angle > SERVO_MAX_STEER + SERVO_MOUNTING_ANGLE) {
        limited_angle = SERVO_MAX_STEER + SERVO_MOUNTING_ANGLE;
    } else if (angle < SERVO_MOUNTING_ANGLE - SERVO_MAX_STEER) {
        limited_angle = SERVO_MOUNTING_ANGLE - SERVO_MAX_STEER;
    }

    double duty_cycle_percent = (12.5 - 2.0)/(M_PI - 0.0) * (limited_angle - 0.0) + 2.0;

    pwm_start(SERVO_PIN_PWM_NAME, SERVO_FREQUENCY_HZ, duty_cycle_percent / 100 * 65536, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  }


  void left_motor_PWM(double duty_cycle_percent) {
    if (duty_cycle_percent > 0) {
      pwm_start(LEFT_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, duty_cycle_percent, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
      pwm_start(LEFT_REVERSE_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, 0, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    } else {
      pwm_start(LEFT_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, 0, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
      pwm_start(LEFT_REVERSE_MOTOR_PIN_PWM_NAME, MOTOR_FREQUENCY_HZ, -duty_cycle_percent, TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }
  }


  void left_motor_steering_drive(double steering_angle, bool reverse, bool inner) {
    if (reverse) {
      left_motor_PWM(-1 * (DEFAULT_MOTOR_DUTY_CYCLE - MOTOR_CORRECTION_SCALING * abs(steering_angle - SERVO_MOUNTING_ANGLE)));
    } else {
      if (inner) {
        left_motor_PWM(DEFAULT_MOTOR_DUTY_CYCLE - MOTOR_CORRECTION_SCALING_INNER * abs(steering_angle - SERVO_MOUNTING_ANGLE));
      } else {
        left_motor_PWM(DEFAULT_MOTOR_DUTY_CYCLE - MOTOR_CORRECTION_SCALING_OUTER * abs(steering_angle - SERVO_MOUNTING_ANGLE));
      }
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


  void right_motor_steering_drive(double steering_angle, bool reverse, bool inner) {
    if (reverse) {
      right_motor_PWM(-1 * (DEFAULT_MOTOR_DUTY_CYCLE - MOTOR_CORRECTION_SCALING * abs(steering_angle - SERVO_MOUNTING_ANGLE)));
    } else {
      if (inner) {
        right_motor_PWM(DEFAULT_MOTOR_DUTY_CYCLE - MOTOR_CORRECTION_SCALING_INNER * abs(steering_angle - SERVO_MOUNTING_ANGLE));
      } else {
        right_motor_PWM(DEFAULT_MOTOR_DUTY_CYCLE - MOTOR_CORRECTION_SCALING_OUTER * abs(steering_angle - SERVO_MOUNTING_ANGLE));
      }    
    }
  }
  
}
