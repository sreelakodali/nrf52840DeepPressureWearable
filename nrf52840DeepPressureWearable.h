#ifndef nrf52840DeepPressureWearable_h
#define nrf52840DeepPressureWearable_h

#include "Arduino.h"
#include <Servo.h>
#include <Wire.h>
#include "LSM6DS3.h"
#include <math.h>

class nrf52840DeepPressureWearable {
	public:
		nrf52840DeepPressureWearable(); // constructor
		void measureRollPitch(bool p);
		void initializeIMU();
	  	void calibrateSensors();
	  	void blinkN(int n, int t_d);
	  	void sweep(int idx, int t_d);

	private:
	  LSM6DS3 myIMU;
	  float accelX, accelY, accelZ;
	  float gyroX, gyroY, gyroZ;
	  float roll, pitch;
	  float complementaryRoll, complementaryPitch;

	  float accelXBias, accelYBias, accelZBias;
	  float gyroXBias, gyroYBias, gyroZBias;
	  unsigned long previousTime;
	  Servo actuatorArr[N_ACT];

	  const  byte I2C_ADDRArr[2] = {0x06, 0x08};
	  const float alpha = 0.98;  // Complementary filter coefficient
	  const int calibrationSamples = 1000;
	  const int T_CYCLE = 15;
	  const int  led_OUT = 13;
	  const  int position_INArr[2] = {A2, A3}; // analog adc pins
	  const int position_OUTArr[2] = {A0, A1}; // pwm output


	  

	  // methods
	  short readDataFromSensor(short address);
	  void readAllAccelGyro();
	  float computeRoll();
	  float computePitch();
	  float complementaryFilter(float v, float dt, bool isRoll);
};


#endif