/*
 * ActuatorSweep
 * Using nrf52840DeepPressureWearable library
 * Written by Sreela Kodali (kodali@stanford.edu) 
 * 
 * */

#include <nrf52840DeepPressureWearable.h>



nrf52840DeepPressureWearable device;


void setup() {
  Serial.begin(9600);
  while (!Serial);
  //device.initializeIMU();
  //device.calibrateSensors();
  device.blinkN(10, 500);
  Serial.println("Device initialized.");
    
}

void loop() {
  device.sweep(0, 500);
  //device.measureRollPitch(1);
}

// ------------------------------- SUPPORT FUNCTIONS --------------------------------//