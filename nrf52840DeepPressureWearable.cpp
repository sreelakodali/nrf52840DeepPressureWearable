// nrf52840DeepPressureWearable.cpp
// Written by: Sreela Kodali, kodali@stanford.edu
#include "nrf52840DeepPressureWearable.h"

nrf52840DeepPressureWearable::nrf52840DeepPressureWearable() {
	LSM6DS3 myIMU(I2C_MODE, 0x6A);
	previousTime = micros();
  pinMode(led_OUT, OUTPUT); // initialize IO

  for (int i=0; i < N_ACT; i++) { // initialize Actuators
    actuatorArr[i].attach(position_OUTArr[i]); // attach servo 
    actuatorArr[i].write(POSITION_MIN); // actuonix
  }
}

void nrf52840DeepPressureWearable::initializeIMU() {
	if (myIMU.begin() != 0) Serial.println("IMU error");
}

void nrf52840DeepPressureWearable::calibrateSensors() {
	float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
  float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;

  for (int i = 0; i < calibrationSamples; i++) {
    sumAccelX += (myIMU).readFloatAccelX();
    sumAccelY += (myIMU).readFloatAccelY();
    sumAccelZ += (myIMU).readFloatAccelZ();
    sumGyroX += (myIMU).readFloatGyroX();
    sumGyroY += (myIMU).readFloatGyroY();
    sumGyroZ += (myIMU).readFloatGyroZ();
    delayMicroseconds(100);
  }

  accelXBias = sumAccelX / calibrationSamples;
  accelYBias = sumAccelY / calibrationSamples;
  accelZBias = sumAccelZ / calibrationSamples - 1.0;
  gyroXBias = sumGyroX / calibrationSamples;
  gyroYBias = sumGyroY / calibrationSamples;
  gyroZBias = sumGyroZ / calibrationSamples;
}

void nrf52840DeepPressureWearable::readAllAccelGyro() {
	accelX = myIMU.readFloatAccelX() - accelXBias;
  accelY = myIMU.readFloatAccelY() - accelYBias;
  accelZ = myIMU.readFloatAccelZ() - accelZBias;
  gyroX = myIMU.readFloatGyroX() - gyroXBias;
  gyroY = myIMU.readFloatGyroY() - gyroYBias;
  gyroZ = myIMU.readFloatGyroZ() - gyroZBias;
}

float nrf52840DeepPressureWearable::computeRoll() {
	return atan2(accelY, accelZ) * 180.0 / M_PI;
}
float nrf52840DeepPressureWearable::computePitch() {
  return atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / M_PI;
}
float nrf52840DeepPressureWearable::complementaryFilter(float v, float dt, bool isRoll) {
  float complementaryValue;

  if (isRoll) complementaryValue = alpha * (complementaryRoll + gyroX * dt) + (1 - alpha) * v;
  else complementaryValue = alpha * (complementaryPitch + gyroY * dt) + (1 - alpha) * v;

  return complementaryValue;
}

void nrf52840DeepPressureWearable::measureRollPitch(bool p) {
	  unsigned long currentTime = micros();
  float deltaTime = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  readAllAccelGyro();
  roll = computeRoll();
  pitch = computePitch();
  complementaryRoll = complementaryFilter(roll, deltaTime, 1);
  complementaryPitch = complementaryFilter(pitch, deltaTime, 0);

  if (p) {
    Serial.print(complementaryRoll, 2);
    Serial.print(",");
    Serial.println(complementaryPitch, 2);
  }
  delay(T_CYCLE);
}

void nrf52840DeepPressureWearable::blinkN(int n, int t_d) {
  for(int i=0; i < n; i++) {
      digitalWrite(led_OUT, LOW);
      delay(t_d/2);
      digitalWrite(led_OUT, HIGH);
      delay(t_d/2);
  }
}

void nrf52840DeepPressureWearable::sweep(int idx, int t_d) {

  for (int i = POSITION_MIN; i < POSITION_MAX; i++) {
    actuatorArr[idx].write(i);
    Serial.println(i);
    delay(t_d);
  }

}

short nrf52840DeepPressureWearable::readDataFromSensor(short address) {
  byte i2cPacketLength = 6;//i2c packet length. Just need 6 bytes from each peripheral
  byte outgoingI2CBuffer[3];//outgoing array buffer
  byte incomingI2CBuffer[6];//incoming array buffer
  bool debug;

  debug = false;

  outgoingI2CBuffer[0] = 0x01;//I2c read command
  outgoingI2CBuffer[1] = 128;//peripheral data offset
  outgoingI2CBuffer[2] = i2cPacketLength;//require 6 bytes

  if (debug) Serial.println("Transmit address");  
  Wire.beginTransmission(address); // transmit to device 
  Wire.write(outgoingI2CBuffer, 3);// send out command
  if (debug) Serial.println("Check sensor status");
  byte error = Wire.endTransmission(); // stop transmitting and check peripheral status
  if (debug) Serial.println("bloop");
  if (error != 0) return -1; //if peripheral not exists or has error, return -1
  Wire.requestFrom((uint8_t)address, i2cPacketLength);//require 6 bytes from peripheral
  if (debug) Serial.println("Request bytes from sensor");
  
  byte incomeCount = 0;
  while (incomeCount < i2cPacketLength)    // peripheral may send less than requested
  {
    if (Wire.available())
    {
      incomingI2CBuffer[incomeCount] = Wire.read(); // receive a byte as character
      incomeCount++;
      if (debug) Serial.println("Read byte from sensor");
    }
    else
    {
      delayMicroseconds(10); //Wait 10us 
      if (debug) Serial.println("Waiting from sensor");
    }
  }

  short rawData = (incomingI2CBuffer[4] << 8) + incomingI2CBuffer[5]; //get the raw data

  return rawData;
}