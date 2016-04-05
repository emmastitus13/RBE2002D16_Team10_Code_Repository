/* Motor Driver Library for driving an H-bridge controlled motor
 * Assumes the following connections 
 * EN  - PWM pin
 * IN1 - Digital IO
 * IN2 - Digital IO
 * Created by Ben Titus, April 5, 2016
 * Last edit made by Ben Titus, April 5, 2016
 */


#ifndef H-BridgeMotorDriver
#define H-BridgeMotorDriver

#include "Arduino.h"
#include <stdlib.h>


//en is enable, in1 and in2 are the two sides of the H-Bridge
H-BridgeMotorDriver::Motor(int en, int in1, int in2) {
	pinMode(pin, OUTPUT);
	enPin = en;
	in1Pin = in1;
	in2Pin = in2;
}


void H-BridgeMotorDriver::driveForward(int spd) {
	digitalWrite(in2Pin, LOW);
	digitalWrite(in1Pin, HIGH);
	analogWrite(enPin, spd);
}


void H-BridgeMotorDriver::driveBackward(int spd) {
	digitalWrite(in1Pin, LOW);
	digitalWrite(in2Pin, HIGH);
	analogWrite(enPin, spd);
}


void H-BridgeMotorDriver::drive(int spd); {
	if (spd < 0) {
		digitalWrite(in1Pin, LOW);
		digitalWrite(in2Pin, HIGH);
		analogWrite(enPin, abs(spd);
	} else {
		digitalWrite(in2Pin, LOW);
		digitalWrite(in1Pin, HIGH);
		analogWrite(enPin, spd);
	}
}


void H-BridgeMotorDriver::brake() {
	digitalWrite(enPin, HIGH);
	digitalWrite(in1Pin, LOW);
	digitalWrite(in2Pin, LOW);
}

#endif