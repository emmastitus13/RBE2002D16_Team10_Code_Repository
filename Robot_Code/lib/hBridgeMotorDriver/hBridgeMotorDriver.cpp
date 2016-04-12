/* Motor Driver Library for driving an H-bridge controlled motor
 * Assumes the following connections
 * IN1 - PWM pin
 * IN2 - PWM pin
 * Created by Ben Titus, April 5, 2016
 * Last edit made by Ben Titus, April 11, 2016
 */


#include "hBridgeMotorDriver.h"



//en is enable, in1 and in2 are the two sides of the H-Bridge
Type1_Motor::Type1_Motor(unsigned char en, unsigned char in1, unsigned char in2) {
	enPin = en;
	in1Pin = in1;
	in2Pin = in2;
	pinMode(enPin, OUTPUT);
	pinMode(in1Pin, OUTPUT);
	pinMode(in2Pin, OUTPUT);
}


void Type1_Motor::driveForward(unsigned char spd) {
	digitalWrite(in2Pin, LOW);
	digitalWrite(in1Pin, HIGH);
	analogWrite(enPin, spd);
}


void Type1_Motor::driveBackward(unsigned char spd) {
	digitalWrite(in1Pin, LOW);
	digitalWrite(in2Pin, HIGH);
	analogWrite(enPin, spd);
}


void Type1_Motor::drive(int spd) {
	bool reverse = false;

	if (spd < 0) {
		reverse = true;
	}

	if (spd > 255) {
		spd = 255;
	}

	if (spd < -255) {
		spd = 255;
	}

	if (reverse) {
		digitalWrite(in1Pin, LOW);
		digitalWrite(in2Pin, HIGH);
	} else {
		digitalWrite(in2Pin, LOW);
		digitalWrite(in1Pin, HIGH);
	}
	digitalWrite(enPin, LOW);
	digitalWrite(in1Pin, !reverse);
	digitalWrite(in2Pin, reverse);
	analogWrite(enPin, spd);
}


void Type1_Motor::brake() {
	digitalWrite(enPin, HIGH);
	digitalWrite(in1Pin, LOW);
	digitalWrite(in2Pin, LOW);
}


/********************************************************************************************************/


//en is enable, in1 and in2 are the two sides of the H-Bridge
Type2_Motor::Type2_Motor(unsigned char in1, unsigned char in2) {
	in1Pin = in1;
	in2Pin = in2;
	pinMode(in1Pin, OUTPUT);
	pinMode(in2Pin, OUTPUT);
}


void Type2_Motor::driveForward(unsigned char spd) {
	digitalWrite(in2Pin, LOW);
	analogWrite(in1Pin, spd);
}


void Type2_Motor::driveBackward(unsigned char spd) {
	digitalWrite(in1Pin, LOW);
	analogWrite(in2Pin, spd);
}


void Type2_Motor::drive(int spd) {
	bool reverse = false;

	if (spd < 0) {
		reverse = true;
	}

	if (spd > 255) {
		spd = 255;
	}

	if (spd < -255) {
		spd = 255;
	}

	if (reverse) {
		digitalWrite(in1Pin, LOW);
		analogWrite(in2Pin, spd);
	} else {
		digitalWrite(in2Pin, LOW);
		analogWrite(in1Pin, spd);
	}
}


void Type2_Motor::brake() {
	digitalWrite(in1Pin, LOW);
	digitalWrite(in2Pin, LOW);
}
