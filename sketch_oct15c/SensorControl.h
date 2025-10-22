#ifndef SENSORCONTROL_H
#define SENSORCONTROL_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

struct Sensor {
  int xshutPin;
  int i2cAddress;
  int index;
};

class SensorControl {
public:
  SensorControl(Sensor left, Sensor mid, Sensor right);
  
  bool begin();
  int readSensorData(int index);
  void printValues(int left, int mid, int right);
  static constexpr int TIMEOUT_VALUE = 0xFFFF;

private:
  bool initOne(Sensor& sensor);
  Sensor sensor_[3];
  VL53L1X devs_[3];
};

#endif