#include "SensorControl.h"
#include "Pins.h"

SensorControl::SensorControl(Sensor left, Sensor mid, Sensor right){
  sensor_[0] = left;
  sensor_[1] = mid;
  sensor_[2] = right;

  for (Sensor& sensor: sensor_) {
    pinMode(sensor.xshutPin, OUTPUT);
    digitalWrite(sensor.xshutPin, LOW);
  }
}

bool SensorControl::begin() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  for (Sensor&  sensorForInit : sensor_) {
      if (!initOne(sensorForInit)) {
          return 0;
      }
  }

  return 1;
}

bool SensorControl::initOne(Sensor& sensor) {
  pinMode(sensor.xshutPin, INPUT);
  delay(20);
  
  devs_[sensor.index].setTimeout(100);

  if (!devs_[sensor.index].init()) {
    Serial.print(F("VL53L1X init failed at index"));
    Serial.println(sensor.index);

    return false;
  }

  devs_[sensor.index].setAddress(sensor.i2cAddress);
  devs_[sensor.index].startContinuous(100);

  return true;
}

int SensorControl::readSensorData(int index) {
  int distance = devs_[index].read();

  if (devs_[index].timeoutOccurred()) {
    return TIMEOUT_VALUE;
  }

  return distance;
}

void SensorControl::printValues(int left, int mid, int right) {
  Serial.print(F("Left: "));

  if (left == TIMEOUT_VALUE) {
    Serial.println(F("Timeout"));
  }
  
  Serial.println(String(left));


  Serial.print(F("Mid: "));

  if (mid == TIMEOUT_VALUE) {
    Serial.println(F("Timeout"));
  }
  
  Serial.println(String(mid));


  Serial.print(F("Right: "));

  if (right == TIMEOUT_VALUE) {
    Serial.println(F("Timeout"));
  }
  
  Serial.println(String(right));
}
