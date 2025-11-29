#include <Arduino.h>
#include "Pins.h"
#include "Config.h"
#include "MotorControl.h"
#include "SensorControl.h"

const int MIN_SPEED = 30;
const int MAX_SPEED = 255 * 0.80;

const float MIN_DIST = 100.0;
const float MAX_DIST = 1000.0;

const float FORWARD_MODE_DIST = 500.0 / MAX_DIST;
const float AVOID_MODE_DIST = 300.0 / MAX_DIST;
const float SEARCH_MODE_DIST = 150.0 / MAX_DIST;

const float SPEED_SMOOTHING_ALPHA = 0.3;

const int DELTA_TIME_MS = 20;

const float Kp = 0.6;
const float Kd = 0.002;

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

float alpha = 0.7;

static float leftSensorDistance = 0.0;
static float midSensorDistance = 0.0;
static float rightSensorDistance = 0.0;

static float leftMotorSpeed = 0;
static float rightMotorSpeed = 0;

static float previous_error = 1.0;

void TaskReadSensors(void *pvParameters) {
  int leftFiltered = 0;
  int midFiltered = 0;
  int rightFiltered = 0;

  for(;;) {
    int l = sensors.readSensorData(left.index);
    int m = sensors.readSensorData(mid.index);
    int r = sensors.readSensorData(right.index);

    leftFiltered = alpha * l + (1 - alpha) * leftFiltered;
    midFiltered = alpha * m + (1 - alpha) * midFiltered;
    rightFiltered = alpha * r + (1 - alpha) * rightFiltered;

    leftFiltered  = constrain(leftFiltered, MIN_DIST, MAX_DIST);
    midFiltered   = constrain(midFiltered, MIN_DIST, MAX_DIST);
    rightFiltered = constrain(rightFiltered, MIN_DIST, MAX_DIST);

    leftSensorDistance   = leftFiltered  / MAX_DIST;
    midSensorDistance    = midFiltered   / MAX_DIST;
    rightSensorDistance  = rightFiltered / MAX_DIST;

    vTaskDelay(pdMS_TO_TICKS(DELTA_TIME_MS));
  }
}

void setup() {
  Serial.begin(115200);

  if (!sensors.begin()) {
    Serial.println(F("Sensors init failed. Halt."));

    while(true) {
      delay(100);
    }
  }

  MotorControl::begin();

  xTaskCreatePinnedToCore(TaskReadSensors, "SensorTask", 4096, NULL, 1, NULL, 0);
}

void loop() {
    float maneuverClearance = (leftSensorDistance * 0.5 + midSensorDistance * 2.0 + rightSensorDistance * 0.5) / 3.0;
    float forwardClearance = pow(midSensorDistance, 1.4);

    float baseSpeed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * forwardClearance;
    float maneuverSpeed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * maneuverClearance;

    State newState = calculateState(state, midSensorDistance);

    Serial.println("Distances: L=" + String(leftSensorDistance) + " M=" + String(midSensorDistance) + " R=" + String(rightSensorDistance));
    Serial.println("Current State: " + String(newState));
    Serial.println("Speeds: Base=" + String(baseSpeed));

    if (state != newState) {
       state = newState;
       stop();
       delay(DELTA_TIME_MS * 5);
    }

    handleState(state, leftSensorDistance, rightSensorDistance, baseSpeed, maneuverSpeed);

    delay(DELTA_TIME_MS);
}


State calculateState(State currentState, float distance) {
    switch (currentState) {
    case FORWARD:
        if (distance < AVOID_MODE_DIST) {
            state = AVOID;
        }

        if (distance < SEARCH_MODE_DIST) {
          state = SEARCH;
        }

        break;
    case AVOID:
        if (distance > FORWARD_MODE_DIST) {
          state = FORWARD;
        }

        if (distance < SEARCH_MODE_DIST) {
          state = SEARCH;
        }

        break;
    case SEARCH:
        if (distance > FORWARD_MODE_DIST) {
          state = FORWARD;
        }

        if (distance > SEARCH_MODE_DIST) {
          state = AVOID;
        }

        break;
    }

    return state;
}

void handleState(State state, float leftDistance, float rightDistance, int baseSpeed, int maneuverSpeed) {
    switch (state) {
      case FORWARD: {
        handleForwardState(leftDistance, rightDistance, baseSpeed);

        break;
      }

      case AVOID: {
        handleAvoidState(leftDistance, rightDistance, maneuverSpeed);

        break;
      }

      case SEARCH: {
        handleSearchState(leftDistance, rightDistance, maneuverSpeed);

        break;
      }
    }
}

void handleForwardState(float leftDistance, float rightDistance, int speed) {
    int leftSpeed = speed;
    int rightSpeed = speed;

    if (leftDistance < rightDistance) {
      float error = leftDistance / rightDistance;
      float derivative = (error - previous_error) / 0.02;

      // turn left
      // rotate right wheel faster forward
      // rotate left wheel slower forward
      leftSpeed = speed * (leftDistance / rightDistance * Kp + derivative * Kd);
      previous_error = error;
    } else {
      float error = rightDistance / leftDistance;
      float derivative = (error - previous_error) / 0.02;

      // turn right
      // rotate right wheel slower forward
      // rotate left wheel faster forward
      rightSpeed = speed * (rightDistance / leftDistance * Kp + derivative * Kd);
      previous_error = error;
    }

    leftSpeed  = constrain(leftSpeed, MIN_SPEED, leftSpeed);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, rightSpeed);

    moveForward(leftSpeed, rightSpeed);
}

void handleAvoidState(float leftDistance, float rightDistance, int speed) {
    if (leftDistance > rightDistance) {
      // left distance is bigger
      // means there is an obstacle on the left
      // turn right
      turnRight(speed, speed);
    } else {
      // right distance is bigger
      // means there is an obstacle on the right
      // turn left
      turnLeft(speed, speed);
    }
}

void handleSearchState(float leftDistance, float rightDistance, int speed) {
    int leftSpeed = speed;
    int rightSpeed = speed;

    // take the maximum current speed and make it proportionally slower
    if (leftDistance < rightDistance) {
       // turn left
       // rotate left wheel faster backwards
       // rotate right wheel slower backwards
       rightSpeed = speed * leftDistance / rightDistance * Kp;
    } else {
       // turn right
       // rotate right wheel faster backwards
       // rotate left wheel slower backwards
       leftSpeed = speed * rightDistance / leftDistance * Kp;
    }

    leftSpeed  = constrain(leftSpeed, MIN_SPEED, leftSpeed);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, rightSpeed);

    moveBackward(leftSpeed, rightSpeed);
}

void moveForward(int targetLeftSpeed, int targetRightSpeed) {
    leftMotorSpeed = smoothenSpeed(leftMotorSpeed, targetLeftSpeed);
    rightMotorSpeed = smoothenSpeed(rightMotorSpeed, targetRightSpeed);

    MotorControl::moveForward(leftMotorSpeed, rightMotorSpeed);
}

void moveBackward(int targetLeftSpeed, int targetRightSpeed) {
    leftMotorSpeed = smoothenSpeed(leftMotorSpeed, targetLeftSpeed);
    rightMotorSpeed = smoothenSpeed(rightMotorSpeed, targetRightSpeed);

    MotorControl::moveBackward(leftMotorSpeed, rightMotorSpeed);
}

void turnLeft(int targetLeftSpeed, int targetRightSpeed) {
    leftMotorSpeed = smoothenSpeed(leftMotorSpeed, targetLeftSpeed);
    rightMotorSpeed = smoothenSpeed(rightMotorSpeed, targetRightSpeed);

    MotorControl::turnLeft(leftMotorSpeed, rightMotorSpeed);
}

void turnRight(int targetLeftSpeed, int targetRightSpeed) {
    leftMotorSpeed = smoothenSpeed(leftMotorSpeed, targetLeftSpeed);
    rightMotorSpeed = smoothenSpeed(rightMotorSpeed, targetRightSpeed);

    MotorControl::turnRight(leftMotorSpeed, rightMotorSpeed);
}

void stop() {
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;

    MotorControl::stop();
}

// take the target speed and smoothen it using exponential moving average
int smoothenSpeed(int currentSpeed, int targetSpeed) {
    return currentSpeed + SPEED_SMOOTHING_ALPHA * (targetSpeed - currentSpeed);
}
