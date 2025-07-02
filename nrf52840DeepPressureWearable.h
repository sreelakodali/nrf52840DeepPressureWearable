#ifndef nrf52840DeepPressureWearable_h
#define nrf52840DeepPressureWearable_h

#include "Arduino.h"
#include <Servo.h>
#include <Wire.h>
#include "LSM6DS3.h"
#include <math.h>
#include "movingAvg.h"
#include <ArduinoBLE.h>


# define N_ACT 2
# define T_SAMPLING 10 // milliseconds
# define WINDOW 10

// actuonix command limits
typedef enum {
	POSITION_MIN = 47, // 900us mightyZap
	POSITION_MAX = 139 // 2100us mightyZap
} ACTUATOR_LIMITS;

class nrf52840DeepPressureWearable {


	public:
		nrf52840DeepPressureWearable(bool serialStatus); // constructor
		void measureRollPitch(bool p);
		void initializeIMU();
	  	void calibrateSensors();
	  	void blinkN(int n, int t_d);
	  	void sweep(int idx, int t_d);
	  	short readDataFromSensor(short address);
	  	void runtime(void (*mapping)(int, int));

	  	Servo actuatorArr[N_ACT];
	  	int position_CommandArr[N_ACT];

	private:
	  // BLEService armPositionService("1840");
	  // BLEFloatCharacteristic armAngles("2763", BLERead | BLENotify);

	  LSM6DS3 myIMU;
	  movingAvg avgFilterPitch;
	  movingAvg avgFilterRoll;
	  float accelX, accelY, accelZ;
	  float gyroX, gyroY, gyroZ;
	  float roll, pitch;
	  float complementaryRoll, complementaryPitch;

	  float accelXBias, accelYBias, accelZBias;
	  float gyroXBias, gyroYBias, gyroZBias;
	  unsigned long previousTime;

	  bool serialON;
	  bool bleON;
	  unsigned long t_lastWrite;

	  const  byte I2C_ADDRArr[2] = {0x06, 0x08};
	  const int T_CYCLE = 15;
	  const int  led_OUT = 13; // 13 green 12 red 14 blue
	  const int  led_R = 12;
	  const int led_B = 14;
	  const  int position_INArr[2] = {A2, A3}; // analog adc pins
	  const int position_OUTArr[2] = {A0, A1}; // pwm output

	  //filtering
	  const float alpha = 0.98;  // Complementary filter coefficient
	  const int calibrationSamples = 1000;



	  
	  // methods
	  void writeOutData(int l, unsigned long t, int *c, int *m, short *d);
	  void readAllAccelGyro();
	  float computeRoll();
	  float computePitch();
	  float complementaryFilter(float v, float dt, bool isRoll);

};

#endif