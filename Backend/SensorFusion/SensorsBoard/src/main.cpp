#include <Arduino.h>
#include <Wire.h>
#include "define.h"
#include "functions.h"


void setup() {
	Serial.begin(115200);
	Wire.begin();
	Wire2.begin();
	delay(1000);

	sensorStart();

	mpuRead();
	getRollPitch();
	kalmanSetup();
}

void loop() {
	allRead();
	writeSerial();
	delay(DELAY_BTWN_READ);
}
