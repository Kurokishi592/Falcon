#include <Arduino.h>
#include <Wire.h>
#include "define.h"
#include "functions.h"


void setup() {
	Serial.begin(9600);
	Wire.begin();
	Wire2.begin();
	delay(1000);

	delay(100);
	mpuRead();
	getRollPitch();
	kalmanSetup();
	setupSW();
}

void loop() {
	allRead();
	writeSerial();
	delay(DELAY_BTWN_READ);
}
