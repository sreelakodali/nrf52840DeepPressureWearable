// nrf52840DeepPressureWearable.cpp
// Written by: Sreela Kodali, kodali@stanford.edu
#include "nrf52840DeepPressureWearable.h"

nrf52840DeepPressureWearable::nrf52840DeepPressureWearable(bool serialStatus): avgFilterPitch(WINDOW), avgFilterRoll(WINDOW) {
  LSM6DS3 myIMU(I2C_MODE, 0x6A);
  previousTime = micros();
  pinMode(led_OUT, OUTPUT); // initialize IO
  pinMode(led_R, OUTPUT); // initialize IO

  // if not(serialStatus) {
  //   BLEService prosthesisService("1840");  // Bluetooth® Low Energy, motorized device
  //   BLEFloatCharacteristic prosthesisCharacteristic("fff1", BLERead | BLEBroadcast);

  //   prosthesisService.addCharacteristic(prosthesisCharacteristic);
  //   BLE.addService(prosthesisService);
  // }

  for (int i=0; i < N_ACT; i++) { // initialize Actuators
    actuatorArr[i].attach(position_OUTArr[i]); // attach servo
    position_CommandArr[i] = POSITION_MIN;
    actuatorArr[i].write(POSITION_MIN); // actuonix
  }
  avgFilterPitch.begin();
  avgFilterRoll.begin();
  serialON = serialStatus;
  bleON = !(serialON);
  t_lastWrite = millis();
  digitalWrite(led_OUT, LOW); // turn LED on

}

void nrf52840DeepPressureWearable::initializeIMU() {
  if (myIMU.begin() != 0) {
    if (serialON) Serial.println("IMU error");
  } 
}

void nrf52840DeepPressureWearable::calibrateSensors() {
  float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
  float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
  digitalWrite(led_OUT, HIGH);
  digitalWrite(led_R, LOW);
  for (int i = 0; i < calibrationSamples; i++) {
    sumAccelX += (myIMU).readFloatAccelX();
    sumAccelY += (myIMU).readFloatAccelY();
    sumAccelZ += (myIMU).readFloatAccelZ();
    sumGyroX += (myIMU).readFloatGyroX();
    sumGyroY += (myIMU).readFloatGyroY();
    sumGyroZ += (myIMU).readFloatGyroZ();
    delayMicroseconds(100);
  }

  digitalWrite(led_R, HIGH);
  digitalWrite(led_OUT, LOW);

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
  float value;
  value= atan2(sqrt(accelY * accelY), sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / M_PI;
  //value = -atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / M_PI;
  return value;
}
float nrf52840DeepPressureWearable::computePitch() {
   // float trueZ = accelZ/ 
  return 180-atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / M_PI;
  //  return (90 + (atan2(accelZ, sqrt(accelX * accelX)) * 180.0 / M_PI));
  //return (90 + (atan2(accelZ, sqrt(accelY * accelY + accelX * accelX)) * 180.0 / M_PI));

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
  //float((*avgFilterR).reading(roll));
  //float((*avgFilterP).reading(pitch));
  complementaryRoll = float(avgFilterRoll.reading(roll));//complementaryFilter(roll, deltaTime, 1);
  complementaryPitch = float(avgFilterPitch.reading(pitch));//pitch;//complementaryFilter(pitch, deltaTime, 0);

  if (p) {
    if (serialON) {
      Serial.print(complementaryRoll, 2);
      Serial.print(",");
      Serial.println(complementaryPitch, 2);
    }
  }
  //delay(T_CYCLE);
}

void nrf52840DeepPressureWearable::blinkN(int n, int t_d) {
  for(int i=0; i < n; i++) {
      digitalWrite(led_OUT, HIGH);
      delay(t_d/2);
      digitalWrite(led_OUT, LOW);
      delay(t_d/2);
  }
}

void nrf52840DeepPressureWearable::sweep(int idx, int t_d) {

  for (int i = POSITION_MIN; i < POSITION_MAX; i++) {
    actuatorArr[idx].write(i);
    if (serialON) Serial.println(i);
    delay(t_d);
  }

}

void nrf52840DeepPressureWearable::writeOutData(int l, unsigned long t, int *c, int *m, short *d) {
  int i;
  String dataString = "";
  if (bleON || serialON) {
    dataString = (String(t) + "," + String(complementaryRoll) + "," + String(complementaryPitch));
    for (i=0; i < l; ++i) dataString += ( "," + String(c[i]) + "," + String(m[i]) + "," + String(d[i]));
    // if (bleON)
    if (serialON) Serial.println(dataString);
  }
}

void nrf52840DeepPressureWearable::runtime(void (*mapping)(int, int)) {
    short data[N_ACT];
    int position_MeasuredArr[N_ACT];
    unsigned long myTime;
    int i;

    myTime = millis(); // 1) time for beginning of the loop
    measureRollPitch(0);// 2) measure latest arm angles
    mapping(int(complementaryPitch), int(complementaryRoll)); // 3) map
    for (i=0; i < N_ACT; ++i) { // 4) bound commands
        if(position_CommandArr[i] > POSITION_MAX) position_CommandArr[i] = POSITION_MAX;
        else if(position_CommandArr[i] < POSITION_MIN) position_CommandArr[i] = POSITION_MIN;
     }

    // 5) Send command to actuator and measure actuator position
    for (i=0; i < N_ACT; ++i) actuatorArr[i].write(position_CommandArr[i]);

    // 6) every T_SAMPLING, read force data, measured position, and print
    if ((myTime - t_lastWrite) > T_SAMPLING) {
      for (i=0; i < N_ACT; ++i) data[i] = readDataFromSensor(I2C_ADDRArr[i]);
      for (i=0; i < N_ACT; ++i) position_MeasuredArr[i] = analogRead(position_INArr[i]);
      writeOutData(N_ACT, myTime, position_CommandArr, position_MeasuredArr, data); // FIX: writeoutData needs to include roll, pitch
      t_lastWrite = millis();
    }

    if (T_CYCLE > 0) delay(T_CYCLE);    
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