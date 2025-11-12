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


// float Ki = 0.0;
// float Kd = 0.2;
// float error = 0;
// float lastError = 0;
// float sumError = 0;

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

const int MIN_SPEED = 0;
const int MAX_SPEED = 255/3;
const int SLOW_SPEED = MAX_SPEED / 3;

const float MIN_DIST = 50.0;
const float MAX_DIST = 2000.0;

const float Kp = 1;

void loop() {
    int leftSensorDistance = sensors.readSensorData(left.index);
    int midSensorDistance = sensors.readSensorData(mid.index);
    int rightSensorDistance = sensors.readSensorData(right.index);

    float leftNorm  = normalizeDistance(leftSensorDistance);
    float midNorm   = normalizeDistance(midSensorDistance);
    float rightNorm = normalizeDistance(rightSensorDistance);

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
        float sensitivity = 1.0 + corridorTightness * 2.0; // 1–3×
        float correction = Kp * error * sensitivity;
        // float correction = (error >= 0 ? sqrt(error) : -sqrt(-error));

        float leftSpeed  = constrain(MAX_SPEED * (1.0 + correction), MIN_SPEED, MAX_SPEED);
        float rightSpeed = constrain(MAX_SPEED * (1.0 - correction), MIN_SPEED, MAX_SPEED);

        // max speed 255 / 3 = 85
        // left distance 30 sm -> normalized 0.15
        // right distance 150 sm -> normalized 0.75
        // error = 0.15 - 0.75 = -0.6 * Kp(1.5) = -0.9
        // left speed = 85 * (1 - 0.9) = 8.5
        // right speed = 85 * (1 + 0.9) = 161.5 -> constrained to 85


        Serial.println("Error: " + String(error) + " Correction: " + String(correction));
        Serial.println("Left speed: " + String(leftSpeed) + " Right speed: " + String(rightSpeed));

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
          MotorControl::turnRight(MAX_SPEED, MAX_SPEED);
        } else {
          MotorControl::turnLeft(MAX_SPEED, MAX_SPEED);
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

        MotorControl::moveBackward(SLOW_SPEED, SLOW_SPEED);

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
