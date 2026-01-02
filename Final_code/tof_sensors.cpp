#include "tof_sensors.h"
#include <Wire.h>

VL53L1X tof1, tof2;

// 定义 XSHUT 引脚
#define XSHUT1 47
#define XSHUT2 21

int tof[2] = {0, 0};

void initXShutPins() {
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
}

void allOff() {
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
}

bool initOne(VL53L1X &sensor, int xshutPin, uint8_t newAddr) {
  digitalWrite(xshutPin, HIGH);
  delay(10);

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("VL53L1X init failed!");
    return false;
  }

  // 改成唯一地址
  sensor.setAddress(newAddr);

  sensor.setDistanceMode(VL53L1X::Medium);
  sensor.setMeasurementTimingBudget(50000);
  return true;
}

void initToF() {
  // ⭐ 关键修改：删除了 Wire.begin()
  // 因为 main.cpp 已经启动了 I2C (40kHz)，这里直接使用即可。

  initXShutPins();
  allOff();
  delay(50);

  // 依次上电 & 改地址 (0x2A, 0x2B)
  if (!initOne(tof1, XSHUT1, 0x2A)) return;
  if (!initOne(tof2, XSHUT2, 0x2B)) return;

  Serial.println("All 2 ToF sensors initialized.");
}

void updateToF() {
  // 读数并做一点偏移校准
  tof[0] = tof1.readRangeSingleMillimeters()-16;
  tof[1] = tof2.readRangeSingleMillimeters()-10;
}