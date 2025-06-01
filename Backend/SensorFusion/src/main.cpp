
#include <Arduino.h>
#include <Wire.h>
#include "define.h"
#include "functions.h"

int count = 0;

void setup() {

	Serial.begin(9600);
	Wire.begin();
	Wire2.begin();
	delay(1000);

	Serial.println("Started");
	//logToSD("HELLO");
	delay(100);
	mpuRead();
	getRollPitch();
	kalmanSetup();
	setupSW();
}

void loop() {
	if (count % 50 == 0) {
		readSW();
		check_reset_CAN();
	}
	if (count == 1000) {
		count = 0;
	}
	allRead();
	delay(DELAY_BTWN_READ);
	count++;

	//Serial.println("Looping"); // For me to count number of loops
}
