#pragma once

#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

// IMU Conversion
void kalmanSetup();
void getRollPitchYawK();

// Mag Conversion
void getCorrectedYaw(float r, float p);

//I2C Scan
// void scanI2CBus();
// void scanI2CDevices();

// Sensor functions
void magRead();
void mpuRead();

void allRead();

void writeSerial();

#endif