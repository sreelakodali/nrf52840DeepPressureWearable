/*
 * IMU Example
 * Using nrf52840DeepPressureWearable library
 * Written by Sreela Kodali (kodali@stanford.edu) and Aarya Sumuk (asumuk@stanford.edu)
 * 
 * */

#include <nrf52840DeepPressureWearable.h>


const bool serialON = true;
nrf52840DeepPressureWearable device(serialON);


void setup() {
  
  if (serialON) {
    Serial.begin(115200);
    while (!Serial);
  }
  device.initializeIMU();
  device.calibrateSensors();
  //device.blinkN(10, 1000);
  if (serialON) Serial.println("Device initialized.");
    
}

void loop() {
  device.measureRollPitch(1);
  delay(15);
}

// ------------------------------- SUPPORT FUNCTIONS --------------------------------//