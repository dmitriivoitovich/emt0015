#include <Arduino.h>
#include "Pins.h"
#include "SensorControl.h"

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

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  if (!sensors.begin()) {
    Serial.println(F("Sensors init failed. Halt."));

    while(true) {
      delay(100);
    }
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second

  //int leftSensorDistance = sensors.readSensorData(left.index);
  //int midSensorDistance = sensors.readSensorData(mid.index);
  //int rightSensorDistance = sensors.readSensorData(right.index);

  //sensors.printValues(leftSensorDistance, midSensorDistance, rightSensorDistance);
  delay(20);
}
