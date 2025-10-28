#include "MotorControl.h"

void MotorControl::begin() {
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR1, OUTPUT);
  pinMode(LEFT_MOTOR_DIR2, OUTPUT);

  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR2, OUTPUT);
}

void MotorControl::stop() {
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
}

void MotorControl::moveForward(int pwm) {
  leftForward(pwm);
  rightForward(pwm);
}

void MotorControl::moveBackward(int pwm) {
  leftBackward(pwm);
  rightBackward(pwm);
}

void MotorControl::turnRight(int leftSpeed, int rightSpeed) {
  leftForward(leftSpeed);
  rightForward(rightSpeed);
}

void MotorControl::turnLeft(int leftSpeed, int rightSpeed) {
  leftForward(leftSpeed);
  rightForward(rightSpeed);
}

void MotorControl::leftForward(int pwm) {
  analogWrite(LEFT_MOTOR_PWM, pwm);
  digitalWrite(LEFT_MOTOR_DIR1, LOW);
  digitalWrite(LEFT_MOTOR_DIR2, HIGH);
}

void MotorControl::leftBackward(int pwm) {
  analogWrite(LEFT_MOTOR_PWM, pwm);
  digitalWrite(LEFT_MOTOR_DIR1, HIGH);
  digitalWrite(LEFT_MOTOR_DIR2, LOW);
}

void MotorControl::rightForward(int pwm) {
  analogWrite(RIGHT_MOTOR_PWM, pwm);
  digitalWrite(RIGHT_MOTOR_DIR1, LOW);
  digitalWrite(RIGHT_MOTOR_DIR2, HIGH);
}

void MotorControl::rightBackward(int pwm) {
  analogWrite(RIGHT_MOTOR_PWM, pwm);
  digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
  digitalWrite(RIGHT_MOTOR_DIR2, LOW);
}
