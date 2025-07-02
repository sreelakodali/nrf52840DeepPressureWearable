/*
 * Runtime Mapping Example
 * Using nrf52840DeepPressureWearable library
 * Written by Sreela Kodali (kodali@stanford.edu) 
 *  NOTE: Actuator indexing starts at 0
 *  Example: for a system with two actuators, first actuator is idx=0. second actuator is idx=1
 * 
 * */

#include <nrf52840DeepPressureWearable.h>
#include <ArduinoBLE.h>

const bool serialON = false;
nrf52840DeepPressureWearable device(serialON);

void setup() {
  Wire.begin();

  if (serialON) {
    Serial.begin(115200);
    while (!Serial);
  } else {
    BLE.begin();
  }
  device.blinkN(10, 500);
  device.initializeIMU();
  device.calibrateSensors();
  if (serialON) Serial.println("Device initialized.");
}

void loop() {
  device.runtime(mapping);
}

// ------------------------------- SUPPORT FUNCTIONS --------------------------------//


// Set position_CommandArr with given arm angle
void mapping(int flex, int rot) {
  int x;
  int y;   
  x = map(flex, 180, 90, POSITION_MIN, POSITION_MAX*0.9);
  y = map(flex, 90, 180, POSITION_MIN, POSITION_MAX*0.9);
  device.position_CommandArr[0] = x;
  device.position_CommandArr[1] = y;
}