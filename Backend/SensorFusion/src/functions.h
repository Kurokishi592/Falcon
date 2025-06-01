#pragma once

#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

// IMU Conversion
void getRollPitch();
void kalmanSetup();
void getRollPitchYawK();

// Mag Conversion
void getYaw();
void getBetterYaw();
void getCorrectedYaw(float r, float p);

// Mag Calibration  
void magCal_withGUI();

//I2C Scan
// void scanI2CBus();
// void scanI2CDevices();

// Sensor read functions
void magRead();
void mpuRead();

void allRead();

#endif