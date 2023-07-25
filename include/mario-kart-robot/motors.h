#pragma once
namespace motors {
    /*
    Purpose: Moves the servo to a specified angle.
    Params:
    - angle: angle that the servo is to be moved to.
    */
    void servo_pwm(double angle);

    /*
    Purpose: Following functions are used to drive the left and right motors respectively.
    Params:
    - duty_cycle_percent: duty cycle of the pwm to the motor (determines motor speed)
    */
    void left_motor_PWM(double duty_cycle_percent);

    
    void left_motor_steering_drive(double steering_angle, bool reverse);
    void right_motor_PWM(double duty_cycle_percent);
    void right_motor_steering_drive(double steering_angle, bool reverse);
}