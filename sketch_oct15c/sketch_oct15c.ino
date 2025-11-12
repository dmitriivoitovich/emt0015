#include <Arduino.h>
#include "Pins.h"
#include "Config.h"
#include "MotorControl.h"
#include "SensorControl.h"

enum State {
  FORWARD,
  AVOID,
  SEARCH
};

Sensor left {
  .xshutPin = 2,
  .i2cAddress = 0x2A,
  .index = 0
};

Sensor mid {
  .xshutPin = 11,
  .i2cAddress = 0x2B,
  .index = 1
};

Sensor right {
  .xshutPin = 10,
  .i2cAddress = 0x2C,
  .index = 2
};

SensorControl sensors(left, mid, right);

State state = FORWARD;

void setup() {
  Serial.begin(115200);

  if (!sensors.begin()) {
    Serial.println(F("Sensors init failed. Halt."));

    while(true) {
      delay(100);
    }
  }

  MotorControl::begin();
}

const int MIN_SPEED = 30;
const int MAX_SPEED = 255;

const float MIN_DIST = 50.0;
const float MAX_DIST = 2000.0;

const float Kp = 0.5;

static float leftFiltered = 0;
static float rightFiltered = 0;
static float midFiltered = 0;
float alpha = 0.7;

void loop() {
    int leftSensorDistance = sensors.readSensorData(left.index);
    int midSensorDistance = sensors.readSensorData(mid.index);
    int rightSensorDistance = sensors.readSensorData(right.index);

    leftFiltered  = alpha * leftSensorDistance  + (1 - alpha) * leftFiltered;
    midFiltered   = alpha * midSensorDistance   + (1 - alpha) * midFiltered;
    rightFiltered = alpha * rightSensorDistance + (1 - alpha) * rightFiltered;

    float leftNorm  = normalizeDistance(leftFiltered);
    float midNorm   = normalizeDistance(midFiltered);
    float rightNorm = normalizeDistance(rightFiltered);

    float clearance = (leftNorm + rightNorm + midNorm) / 3.0;
    float baseSpeed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * clearance;
    float maneuverSpeed = constrain(baseSpeed / 2, MIN_SPEED * 2, MAX_SPEED / 2);

    Serial.println("Distances: L=" + String(leftSensorDistance) + " (" + String(leftNorm) + ") M=" + String(midSensorDistance) + " (" + String(midNorm) + ") R=" + String(rightSensorDistance) + " (" + String(rightNorm) + ")");
    Serial.println("State: " + String(state));

    switch (state) {
      case FORWARD: {
        // if the mid sensor detects an obstacle closer than 15% of its range (30 sm), switch to AVOID state
        if (midNorm < 0.15) {
          MotorControl::stop();

          state = AVOID;

          break;
        }

        // if the mid sensor detects an obstacle closer than 5% of its range (10 sm), switch to SEARCH state
        if (midNorm < 0.05) {
          state = SEARCH;

          break;
        }

        // error is the difference between left and right distances
        // if error is positive, the distance to the right obstacle is bigger than to the left one, and we need to slightly turn right
        // if error is negative, the distance to the left obstacle is bigger than to the right one, and we need to slightly turn left
        float error = leftNorm - rightNorm;

        float corridorTightness = (MAX_DIST - ((leftSensorDistance + rightSensorDistance) / 2.0)) / MAX_DIST;
        float sensitivity = 1.0 + corridorTightness * 2.0;
        float correction = Kp * error * sensitivity;
        // float correction = (error >= 0 ? sqrt(error) : -sqrt(-error));

        float leftSpeed  = constrain(baseSpeed * (1.0 + correction), MIN_SPEED, baseSpeed);
        float rightSpeed = constrain(baseSpeed * (1.0 - correction), MIN_SPEED, baseSpeed);

        Serial.printf(
          "{L:%d M:%d R:%d e:%.2f c:%.2f s:%d}\n",
          leftSensorDistance,
          midSensorDistance,
          rightSensorDistance,
          error,
          correction,
          state
        );

        MotorControl::moveForward(leftSpeed, rightSpeed);

        break;
      }

      case AVOID: {
        if (midNorm > 0.20) {
          state = FORWARD;

          break;
        }

        if (midNorm < 0.05) {
          MotorControl::stop();

          state = SEARCH;

          break;
        }

        if (leftNorm > rightNorm) {
          MotorControl::turnRight(maneuverSpeed, maneuverSpeed);
        } else {
          MotorControl::turnLeft(maneuverSpeed, maneuverSpeed);
        }

        break;
      }

      case SEARCH: {
        if (midNorm > 0.20) {
          state = FORWARD;

          break;
        }

        if (midNorm > 0.10) {
          MotorControl::stop();

          state = AVOID;

          break;
        }

        MotorControl::moveBackward(maneuverSpeed, maneuverSpeed);

        break;
      }
    }

    delay(20);
}

// normalize distance MIN_DIST .. MAX_DIST to range 0.0 .. 1.0
float normalizeDistance(int distance) {
  distance = constrain(distance, MIN_DIST, MAX_DIST);

  return (distance - MIN_DIST) / (MAX_DIST - MIN_DIST);
}
