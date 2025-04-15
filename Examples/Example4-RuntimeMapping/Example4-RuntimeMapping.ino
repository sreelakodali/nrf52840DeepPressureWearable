/*
 * Runtime Mapping Example
 * Using nrf52840DeepPressureWearable library
 * Written by Sreela Kodali (kodali@stanford.edu) 
 *  NOTE: Actuator indexing starts at 0
 *  Example: for a system with two actuators, first actuator is idx=0. second actuator is idx=1
 * 
 * */

#include <nrf52840DeepPressureWearable.h>
const bool serialON = true;
nrf52840DeepPressureWearable device(serialON);

void setup() {
  Wire.begin();

  if (serialON) {
    Serial.begin(9600);
    while (!Serial);
  }
  device.initializeIMU();
  device.calibrateSensors();
  device.blinkN(10, 500);
  if (serialON) Serial.println("Device initialized.");
}

void loop() {
  device.runtime(mapping);
}

// ------------------------------- SUPPORT FUNCTIONS --------------------------------//


// Set position_CommandArr with given arm angle
void mapping(int angle) {
  int x;   
  x = map(angle, 0, 90, POSITION_MIN, POSITION_MAX);
  device.position_CommandArr[0] = x;
  device.position_CommandArr[1] = x;
}