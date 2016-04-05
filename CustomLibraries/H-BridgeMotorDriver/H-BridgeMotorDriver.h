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


class Motor {
  public:
    Motor(int en, int in1, int in2);
    void driveForward(int spd);
	void driveBackward(int spd);
	void drive(int spd);
	void brake();

  private:
    int enPin, in1Pin, in2Pin;
};


#endif