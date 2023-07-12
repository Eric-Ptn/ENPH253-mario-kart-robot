#include <Arduino.h>
#include <Servo.h>

/*
This is just an initial attempt to code the control system for tape following (I would try to make this better but sick)
Potential improvements:
- Add a history for the error (perhaps an array that has previous errors, if the error is increasing, steer harder)
- Maybe alter this to work with only 2 IR sensors (maybe this would make the code run faster, but honestly, I don't know if it will
  matter much)
*/

//change to the actual pins later
#define SERVO_PIN 9 
#define IR_SENSOR_0 0
#define IR_SENSOR_1 1
#define IR_SENSOR_2 2
#define IR_SENSOR_3 3



Servo servo; 

//variables for PID control
int error = 0;
int last_error = 0;
int integral = 0;
int derivative = 0;
int proportional = 0;
int ir_readings[] = {0,0,0,0};

//placeholder values, will need to do tuning to determine properly
int Kp = 1;
int Ki = 0;
int Kd = 124321;


void setup() {
  // put your setup code here, to run once:
  servo.attach(SERVO_PIN);
  pinMode(IR_SENSOR_0, INPUT_PULLUP);
  pinMode(IR_SENSOR_1, INPUT_PULLUP);
  pinMode(IR_SENSOR_2, INPUT_PULLUP);
  pinMode(IR_SENSOR_3, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  ir_readings[0] = digitalRead(IR_SENSOR_0);
  ir_readings[1] = digitalRead(IR_SENSOR_1);
  ir_readings[2] = digitalRead(IR_SENSOR_2);
  ir_readings[3] = digitalRead(IR_SENSOR_3);

  //This is a really jank way of assigning errors

  if (ir_readings[0] == HIGH && ir_readings[1] == LOW) {
    error = -2;
  }
  else if (ir_readings[0] == HIGH && ir_readings[1] == HIGH) {
    error = -1;
  }
  else if (ir_readings[1] == HIGH && ir_readings[2] == HIGH) {
    error = 0;
  }
  else if (ir_readings[2] == HIGH && ir_readings[3] == HIGH) { 
    error = 1;
  }
  else if (ir_readings[2] == LOW && ir_readings[3] == HIGH) {
    error = 2;
  }

  derivative = last_error - error;
  integral += error; //add wind-up safety to this? 
  proportional = error;
  last_error = error;

  int correction_val = Kd * derivative + Ki*integral + Kp*proportional;

  //assumes that the servo is mounted at 90 degrees 
  servo.write(90 - correction_val);
  
}

// put function definitions here:
