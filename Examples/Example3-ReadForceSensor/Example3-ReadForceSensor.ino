/*
 * ReadForceSensor
 * Using nrf52840DeepPressureWearable library
 * Written by Sreela Kodali (kodali@stanford.edu) 
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
  //device.initializeIMU();
  //device.calibrateSensors();
  device.blinkN(10, 500);
  if (serialON) Serial.println("Device initialized.");
    
}

void loop() {

  byte i2cAddress = 0x06; // Peripheral address (SingleTact), default 0x04
  short data = device.readDataFromSensor(i2cAddress);
  if (serialON) Serial.println(data);  
  delay(15); // Change this if you are getting values too quickly 

  //device.sweep(0, 500);
  //device.measureRollPitch(1);
}

// ------------------------------- SUPPORT FUNCTIONS --------------------------------//