#include <Arduino.h>
#include <math.h>
#include <mario-kart-robot\motors.h>
#include <mario-kart-robot\config.h>
#include <mario-kart-robot\oled_display.h>

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

    // https://www.amazon.ca/ANNIMOS-Coreless-Stainless-Waterproof-Standard/dp/B07SSNJ6Y5?th=1
    // double microseconds = (2500 - 500)/(M_PI * 3 / 2 - 0.0) * (limited_angle - 0.0) + 500;
    // pwm_start(SERVO_PIN_PWM_NAME, SERVO_FREQUENCY_HZ, microseconds, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

    double duty_cycle_percent = (12.5 - 2.5)/(3 * M_PI / 2.0) * (limited_angle - 0.0) + 2.5;
    int val = duty_cycle_percent / 100 * 65536;

    pwm_start(SERVO_PIN_PWM_NAME, SERVO_FREQUENCY_HZ, val, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
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


  void left_motor_steering_drive(double steering_angle, bool reverse, double constant_offset) {
    double adjusted_value = steering_duty_cycle(steering_angle, steering_angle < SERVO_MOUNTING_ANGLE, constant_offset);
    if (reverse) {
      left_motor_PWM(-1 * adjusted_value);
    } else {
      left_motor_PWM(adjusted_value);
    }
    // OLED::display_text(String(adjusted_value) + " Outer: " + String(steering_angle < SERVO_MOUNTING_ANGLE) + " Steering angle: " + String(steering_angle));
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


  void right_motor_steering_drive(double steering_angle, bool reverse, double constant_offset) {
    double adjusted_value = steering_duty_cycle(steering_angle, steering_angle > SERVO_MOUNTING_ANGLE, constant_offset);
    if (reverse) {
      right_motor_PWM(-1 * adjusted_value);
    } else {
      right_motor_PWM(adjusted_value);
    }
    // OLED::display_text(String(adjusted_value));
  }

  // https://math.stackexchange.com/questions/2655178
  // function to take in steering angle and output duty cycle based on parabolic function

  // NOTE: steering_angle can be negative!
  // see https://www.desmos.com/calculator/ad7njfzqat

  double steering_duty_cycle(double steering_angle, bool outer, double constant_offset) {
    double abs_angle = abs(steering_angle - SERVO_MOUNTING_ANGLE);

    // first point: default duty cycle when steering angle is SERVO_MOUNTING_ANGLE (SERVO_MOUNTING_ANGLE, DEFAULT_MOTOR_DUTY_CYCLE)
    double f1 = (abs_angle - SERVO_MAX_STEER) * (abs_angle - (REDUCTION_DUTY_CYCLE_ANGLE - SERVO_MOUNTING_ANGLE)) / ((0 - SERVO_MAX_STEER) * (0 - (REDUCTION_DUTY_CYCLE_ANGLE - SERVO_MOUNTING_ANGLE)));

    // second point: duty cycle when steering angle is at min duty cycle angle
    double f2 = (abs_angle - SERVO_MAX_STEER) * (abs_angle - 0) / (((REDUCTION_DUTY_CYCLE_ANGLE - SERVO_MOUNTING_ANGLE) - 0) * ((REDUCTION_DUTY_CYCLE_ANGLE - SERVO_MOUNTING_ANGLE) - SERVO_MAX_STEER));

    // third point: duty cycle when steering angle is max
    double f3 = (abs_angle - (REDUCTION_DUTY_CYCLE_ANGLE - SERVO_MOUNTING_ANGLE)) * (abs_angle - 0) / ((SERVO_MAX_STEER - 0) * (SERVO_MAX_STEER - (REDUCTION_DUTY_CYCLE_ANGLE - SERVO_MOUNTING_ANGLE)));

    if (outer) {
      return DEFAULT_MOTOR_DUTY_CYCLE * f1 + (DEFAULT_MOTOR_DUTY_CYCLE - DUTY_CYCLE_REDUCTION_OUTER) * f2 + (DEFAULT_MOTOR_DUTY_CYCLE + MAX_DUTY_CYCLE_BOOST_OUTER) * f3 + constant_offset;
    }
    return DEFAULT_MOTOR_DUTY_CYCLE * f1 + (DEFAULT_MOTOR_DUTY_CYCLE - DUTY_CYCLE_REDUCTION_INNER) * f2 + (DEFAULT_MOTOR_DUTY_CYCLE + MAX_DUTY_CYCLE_BOOST_INNER) * f3 + constant_offset;

  }

}
