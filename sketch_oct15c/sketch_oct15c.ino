#include <Arduino.h>
#include "Pins.h"
#include "Config.h"
#include "MotorControl.h"
#include "SensorControl.h"

const int MIN_SPEED = 30;
const int MAX_SPEED = 255;

const float MIN_DIST = 100.0;
const float MAX_DIST = 1000.0;

const float FORWARD_MODE_DIST = 500.0 / MAX_DIST;
const float AVOID_MODE_DIST = 300.0 / MAX_DIST;
const float SEARCH_MODE_DIST = 150.0 / MAX_DIST;

const float Kp = 0.7;

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

//     Serial.println("Raw Distances: L=" + String(l) + " M=" + String(m) + " R=" + String(r));

    leftFiltered  = constrain(leftFiltered, MIN_DIST, MAX_DIST);
    midFiltered   = constrain(midFiltered, MIN_DIST, MAX_DIST);
    rightFiltered = constrain(rightFiltered, MIN_DIST, MAX_DIST);

//     Serial.println("Filtered Distances: L=" + String(leftFiltered) + " M=" + String(midFiltered) + " R=" + String(rightFiltered));

    leftSensorDistance   = leftFiltered  / MAX_DIST;
    midSensorDistance    = midFiltered   / MAX_DIST;
    rightSensorDistance  = rightFiltered / MAX_DIST;

//     Serial.println("Normalized Distances: L=" + String(leftSensorDistance) + " M=" + String(midSensorDistance) + " R=" + String(rightSensorDistance));

    vTaskDelay(pdMS_TO_TICKS(20));
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
    float clearance = (leftSensorDistance + rightSensorDistance + midSensorDistance) / 3.0;

    // base speed multiplied by clearance square root
    float baseSpeed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * sqrt(clearance);
    float maneuverSpeed = constrain(baseSpeed / 2, MIN_SPEED, MAX_SPEED / 2);

    State newState = calculate_state(state, midSensorDistance);

    Serial.println("Distances: L=" + String(leftSensorDistance) + " M=" + String(midSensorDistance) + " R=" + String(rightSensorDistance));
    Serial.println("Current State: " + String(newState));
    Serial.println("Speeds: Base=" + String(baseSpeed) + " Maneuver=" + String(maneuverSpeed));

    if (state == FORWARD && newState != FORWARD) {
       MotorControl::stop();
         delay(20);
    }

    switch (newState) {
      case FORWARD: {
        handle_forward_state(leftSensorDistance, rightSensorDistance, baseSpeed);

        break;
      }

      case AVOID: {
        handle_avoid_state(leftSensorDistance, rightSensorDistance, baseSpeed);

        break;
      }

      case SEARCH: {
        handle_search_state(leftSensorDistance, rightSensorDistance, baseSpeed);

        break;
      }
    }

    state = newState;

    delay(20);
}

State calculate_state(State currentState, float distance) {
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

void handle_forward_state(float leftDistance, float rightDistance, int speed) {
    int leftSpeed = speed;
    int rightSpeed = speed;

    if (leftDistance < rightDistance) {
      // turn left
      // rotate right wheel faster forward
      // rotate left wheel slower forward
      leftSpeed = speed * leftDistance / rightDistance * Kp;
    } else {
      // turn right
      // rotate right wheel slower forward
      // rotate left wheel faster forward
      rightSpeed = speed * rightDistance / leftDistance * Kp;
    }

    leftSpeed  = constrain(leftSpeed, MIN_SPEED, leftSpeed);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, leftSpeed);

    MotorControl::moveForward(leftSpeed, rightSpeed);
}

void handle_avoid_state(float leftDistance, float rightDistance, int speed) {
    if (leftDistance > rightDistance) {
      // left distance is bigger
      // means there is an obstacle on the left
      // turn right
      MotorControl::turnRight(speed, speed);
    } else {
      // right distance is bigger
      // means there is an obstacle on the right
      // turn left
      MotorControl::turnLeft(speed, speed);
    }
}

void handle_search_state(float leftDistance, float rightDistance, int speed) {
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
    rightSpeed = constrain(rightSpeed, MIN_SPEED, leftSpeed);

    MotorControl::moveBackward(leftSpeed, rightSpeed);
}
