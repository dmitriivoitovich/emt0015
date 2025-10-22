#include <Arduino.h>
#include "Config.h"
#include "MotorControl.h"

void setup() {
  Serial.begin(115200);

  MotorControl::begin();

  Serial.println(F("Motor test start"));
}

void loop() {
  int FWD_PWM = 255;
  int BWD_PWM = 255;
  int FAST = 255;
  int SLOW = 100;

  // int FWD_PWM = 150;
  // int BWD_PWM = 125;
  // int FAST = 200;
  // int SLOW = 100;

  Serial.println(F("Forward 5s"));
  MotorControl::moveForward(FWD_PWM);
  delay(5000);

  MotorControl::stop();
  delay(1000);

  Serial.println(F("Turn right 5s"));

  MotorControl::turnRight(FAST, SLOW);
  delay(5000);

  Serial.println(F("Cycle done"));
}
