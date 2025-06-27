#include "define.h"
#include "functions.h"

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <MPU6500_WE.h>
#include <new_kalman/new_kalman.h>
#include <SPI.h>
#include <string>

Adafruit_LIS3MDL MAG;
MPU6500_WE MPU = MPU6500_WE(&Wire2, MPU6500_ADDR);

bool MAG_E = true, MPU_E = true;
bool MAG_C = false, MPU_C = false;

// MPU raw data
double accelX = 9, accelY = 9, accelZ = 9;
double gyroX = 9, gyroY = 9, gyroZ = 9;

// MPU processed data
double roll = 9, pitch = 9;
Kalman kalmanX, kalmanY, kalmanZ;
uint32_t timer = 0;
float kalPitch = 0, kalRoll = 0, kalYaw = 0;

// MAG raw data
double magX = 9, magY = 9, magZ = 9;

// MAG processed data
double yaw = 9;

double temp = 9;

void getRollPitch() {
	roll = atan((double)accelY / hypotenuse((double)accelX, (double)accelZ)) * RAD_TO_DEG;
	pitch = atan2((double)-accelX, (double)accelZ) * RAD_TO_DEG;
}

void sensorStart() {
	if (MAG_E) { magSetup(); if (MAG_C) { PRINTLN("LIS3MDL setup done"); } }
	if (MPU_E) { mpuSetup(); if (MPU_C) { PRINTLN("MPU6500 setup done"); } }
}

/**
 * Kalman Filter TKJ setup
 */
void kalmanSetup() {
	kalmanX.setAngle(roll);
	kalmanY.setAngle(pitch);
	kalmanZ.setAngle(yaw);
	timer = micros();
	kalmanX.setQbias(0.002f);
	kalmanX.setQangle(0.001f);
	kalmanY.setQbias(0.002f);
	kalmanY.setQangle(0.001f);
	kalmanZ.setQbias(0.003f);
	kalmanZ.setQangle(0.001f);
	kalmanZ.setRmeasure(0.03f);
}

/**
 * LIS3MDL setup process
 */
void magSetup() {
	int count = 0;

	while (!MAG.begin_SPI(CS_MAG, CLK, MISO, MOSI) && count < 10) { // soft SPI
		PRINTLN("Failed to find LIS3MDL chip");
		count += 1;
		delay(100);
	}
	if (count < 10) { PRINTLN("MAG connected"); MAG_C = true; }
	else { PRINTLN("MAG NOT connected"); }

	if (MAG_C) {
		MAG.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
		PRINT("Performance mode set to: ");
		switch (MAG.getPerformanceMode()) {
			case LIS3MDL_LOWPOWERMODE:
				PRINTLN("Low");
				break;
			case LIS3MDL_MEDIUMMODE:
				PRINTLN("Medium");
				break;
			case LIS3MDL_HIGHMODE:
				PRINTLN("High");
				break;
			case LIS3MDL_ULTRAHIGHMODE:
				PRINTLN("Ultra-High");
				break;
		}

		MAG.setOperationMode(LIS3MDL_CONTINUOUSMODE);
		PRINT("Operation mode set to: ");
		// Single shot mode will complete conversion and go into power down
		switch (MAG.getOperationMode()) {
			case LIS3MDL_CONTINUOUSMODE:
				PRINTLN("Continuous");
				break;
			case LIS3MDL_SINGLEMODE:
				PRINTLN("Single mode");
				break;
			case LIS3MDL_POWERDOWNMODE:
				PRINTLN("Power-down");
				break;
		}

		MAG.setDataRate(LIS3MDL_DATARATE_155_HZ);
		// You can check the datarate by looking at the frequency of the DRDY pin
		PRINT("Data rate set to: ");
		switch (MAG.getDataRate()) {
			case LIS3MDL_DATARATE_0_625_HZ:
				PRINTLN("0.625 Hz");
				break;
			case LIS3MDL_DATARATE_1_25_HZ:
				PRINTLN("1.25 Hz");
				break;
			case LIS3MDL_DATARATE_2_5_HZ:
				PRINTLN("2.5 Hz");
				break;
			case LIS3MDL_DATARATE_5_HZ:
				PRINTLN("5 Hz");
				break;
			case LIS3MDL_DATARATE_10_HZ:
				PRINTLN("10 Hz");
				break;
			case LIS3MDL_DATARATE_20_HZ:
				PRINTLN("20 Hz");
				break;
			case LIS3MDL_DATARATE_40_HZ:
				PRINTLN("40 Hz");
				break;
			case LIS3MDL_DATARATE_80_HZ:
				PRINTLN("80 Hz");
				break;
			case LIS3MDL_DATARATE_155_HZ:
				PRINTLN("155 Hz");
				break;
			case LIS3MDL_DATARATE_300_HZ:
				PRINTLN("300 Hz");
				break;
			case LIS3MDL_DATARATE_560_HZ:
				PRINTLN("560 Hz");
				break;
			case LIS3MDL_DATARATE_1000_HZ:
				PRINTLN("1000 Hz");
				break;
		}

		MAG.setRange(LIS3MDL_RANGE_4_GAUSS);
		PRINT("Range set to: ");
		switch (MAG.getRange()) {
			case LIS3MDL_RANGE_4_GAUSS:
				PRINTLN("+-4 gauss");
				break;
			case LIS3MDL_RANGE_8_GAUSS:
				PRINTLN("+-8 gauss");
				break;
			case LIS3MDL_RANGE_12_GAUSS:
				PRINTLN("+-12 gauss");
				break;
			case LIS3MDL_RANGE_16_GAUSS:
				PRINTLN("+-16 gauss");
				break;
		}

		MAG.setIntThreshold(500);
		MAG.configInterrupt(false, false, true, // enable z axis
							true,				// polarity
							false,				// don't latch
							true);				// enabled!
	}
}

/**
 * MPU6500 setup process
 */
void mpuSetup() {
	int count = 0;
	while (!MPU.init() && count < 10) {
		PRINTLN("MPU6500 does not respond");
		delay(100);
		count += 1;
	}
	if (count < 10) {
		PRINTLN("MPU6500 is connected");
		MPU_C = true;
		MPU.enableGyrDLPF();
		MPU.setGyrDLPF(MPU6500_DLPF_6);
		MPU.setSampleRateDivider(5);
		MPU.setGyrRange(MPU6500_GYRO_RANGE_250);
		MPU.setAccRange(MPU6500_ACC_RANGE_2G);
		MPU.enableAccDLPF(true);
		MPU.setAccDLPF(MPU6500_DLPF_6);
		mpuCalib();
	}
	else if (count > 10) { PRINTLN("MPU6500 is NOT connected"); PRINTLN("I died"); }
}

/**
 * MPU6500 auto calibration process
 */
void mpuCalib() {
	PRINTLN("Position you MPU6500 flat and don't move it - calibrating...");
	delay(1000);
	MPU.autoOffsets();
	PRINTLN("MPU Calibration Done!");
}

/**
 * Function to get LIS3MDL (MAG) readings
 */
void magRead() {
	/* get a new sensor event, normalized to uTesla */
	sensors_event_t event;
	MAG.getEvent(&event);
	/* Display the results (magnetic field is measured in uTesla) */
	
	magX = 1.0*(event.magnetic.x) - 3.05;
	magY = 1.0*(event.magnetic.y) + 31.925;
	magZ = 1.11273*(event.magnetic.z) + 20.07273;

	float scale = 52 / sqrt(magX * magX + magY * magY + magZ * magZ);

	magX *= scale;
	magY *= scale;
	magZ *= scale;

	PRINTLN("\nMAG Reading:\nX: " + String(event.magnetic.x) + "\tY: " + String(event.magnetic.y) + "\tZ: " + String(event.magnetic.z) + " uTesla");
	PRINTLN("offsetted X: " + String(magX) + "\toffsetted Y: " + String(magY) + "\toffsetted Z: " + String(magZ) + " uTesla");
}

/**
 * Function to get MPU6500 readings
 */
void mpuRead() {
	xyzFloat gValue = MPU.getGValues();
	xyzFloat gyr = MPU.getGyrValues();
	float temperature = MPU.getTemperature();
	float resultantG = MPU.getResultantG(gValue);

	accelX = gValue.x; //- accelX_offsetGlobal;
	accelY = gValue.y; //- accelY_offsetGlobal;
	accelZ = gValue.z; //- accelZ_offsetGlobal;
	gyroX = gyr.x; //- gyroX_offsetGlobal;
	gyroY = gyr.y; //- gyroY_offsetGlobal;
	gyroZ = gyr.z; //- gyroZ_offsetGlobal;
	temp = temperature;

	PRINTLN("\nMPU Reading:\nAcceleration in g (x,y,z):\t" + String(accelX) + "   " + String(accelY) + "   " + String(accelZ) + "\nResultant g: " + String(resultantG));
	PRINTLN("Gyroscope data in degrees/s:\t" + String(gyroX) + "   " + String(gyroY) + "   " + String(gyroZ));
	PRINTLN("Temperature in °C: " + String(temperature));
	PRINTLN("********************************************");
}

/**
 * Get yaw values by rotating the mag readings back into the reference frame of the accel/gyro, to remove the influence of roll on yaw.
 */
void getCorrectedYaw(float r, float p) {	
	// convert roll and pitch to radians
	r *= M_PI / 180.0f;
	p *= M_PI / 180.0f;
	float original_magX = magX;
	float original_magY = magY;
	float original_magZ = magZ;

	//for roll
	float polar_r_xz = sqrt(magX*magX + magZ*magZ);
	float theta_xz_init = atan2(magX,magZ);
	float polar_angle_after_roll = theta_xz_init - r;
	magX = polar_r_xz * cos(r);
	magZ = polar_r_xz * sin(r);
	
	//for pitch
	float polar_r_yz = sqrt(magY*magY + magZ*magZ);
	float theta_yz_init = atan2(magY,magZ);
	float polar_angle_after_pitch = theta_yz_init - p;
	magY = polar_r_yz * cos(p);
	magZ = polar_r_yz * sin(p);

	float deltaX = magX - original_magX;
	float deltaY = magY - original_magY;

	yaw = atan2(magX - deltaX, magY - deltaY) * 180/M_PI;
}

/**
 * Final Roll Pitch and Yaw calculation with Kalman Filter
*/
void getRollPitchYawK() {
	double dt = (double)(micros() - timer) / 1000000;
	timer = micros();
	kalRoll = kalmanX.getAngle(roll, gyroX, dt);
	kalPitch = kalmanY.getAngle(pitch, gyroY, dt);
	getCorrectedYaw(kalRoll, kalPitch);
	kalYaw = kalmanZ.getAngle(yaw, gyroZ, dt);
	PRINTLN(" kalYaw: " + String(kalYaw));
	
	// Serial.printf("kalRoll: %.7f, kalPitch: %.7f, kalYaw: %.7f\n", kalRoll, kalPitch, kalYaw);
	// Serial.println("Orientation: " + String(kalRoll) + ", " + String(kalPitch) + ", " + String(kalYaw));
}


void allRead() {
	if (MAG_C) { magRead(); delay(READ_DELAY); }
	if (MPU_C) { mpuRead(); getRollPitch(); getRollPitchYawK(); delay(READ_DELAY); }
}

void writeSerial() {
	Serial.print("Roll:");
	Serial.println(kalRoll);
	Serial.print("Pitch:");
	Serial.println(kalPitch);
	Serial.print("Yaw:");
	Serial.println(kalYaw);
	Serial.print("Temp:");
	Serial.println(temp);
}