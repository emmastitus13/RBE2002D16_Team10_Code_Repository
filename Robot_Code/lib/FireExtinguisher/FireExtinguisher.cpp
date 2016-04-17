/* Fire Extinguisher using a propeller on a pan and tilt bracket with an infrared flame sensor
 * Uses 1 fan motor, 1 micro servo, and 1 infrared flame sensor
 * Pins
 * FAN_PIN - 1 Digital I/O
 * TILT_SERVO - 1 PWM Port
 * FLAME_SENSOR - 1 Analog Input, 1 Digital Input
 *
 * Created on Apr 12, 2016 by Ben Titus
 * Last edit made Apr 16, 2016 by Ben Titus
 */

#include "FireExtinguisher.h"
#include <Servo.h>

FireExtinguisher::FireExtinguisher(uint8_t fan, uint8_t flameSenseA, uint8_t flameSenseD, uint8_t servo, int flameConst) {
    fanPin = fan;
    flameSensePinA = flameSenseA;
    flameSensePinD = flameSenseD;
    servoPin = servo;
    flameSensorConstant = flameConst;
    Servo myServo;
    tiltServo = myServo;
}


//sets the min and max values for the tilt servo
void FireExtinguisher::setServo(uint8_t min, uint8_t max) {
    tiltServo.attach(servoPin);
    servoMin = min;
    servoMax = max;
    servoPos = 0;
}


//turns on the fan and reads the flame sensor until the flame is extinguished
void FireExtinguisher::extinguishFire(void) {
    int dist = readFlameSense();
    if (dist > 600) {
        digitalWrite(fanPin, LOW);
    } else {
        digitalWrite(fanPin, HIGH);
    }
}


//returns the analogRead() value from the flame sensor
//value should be lower the more flame is sensed
int FireExtinguisher::readFlameSense(void) {
    return analogRead(flameSensePinA);
}


//returns the digitalRead() value from the flame sensor
//sensitivity can be adjusted by turning the potentiometer on the flame sensor
int FireExtinguisher::readFlameSenseDig(void) {
    return digitalRead(flameSensePinD);
}


//returns a crude distance to the flame based on the calibration curve of the flame sensor
//should not be trusted as 100% accurrate
int FireExtinguisher::getDistance(void) {
    int val = analogRead(flameSensePinA);
    return val / flameSensorConstant;
}


//tilts the servo to an angle, causing the array to tilt up and down
void FireExtinguisher::servoTilt(int tiltTo) {
    if (tiltTo > 255) {
        tiltTo = 255;
    }
    if (tiltTo < 0) {
        tiltTo = 0;
    }
    tiltServo.write(tiltTo);
    servoPos = tiltTo;
}


//moves the servo up and down to find the flame
int FireExtinguisher::findFlame(void) {
    servoTilt(servoMin); //reset the tilt servo to minimum position

    int pastFlameVal, flameVal;
    pastFlameVal = 1024; //set the past value to maximum
    flameVal = readFlameSense(); //get the current flame sensor value

    for (int i = servoMin; i < servoMax; i++) {
        if (flameVal > pastFlameVal) {
            return i;
        } else {
            pastFlameVal = flameVal;
            flameVal = readFlameSense();
            servoTilt(i);
        }
    }
}


//turns on the fan
void FireExtinguisher::fanOn(void) {
    if (servoPos < 10) {} else {
        digitalWrite(fanPin, HIGH);
    }
}


//turns off the fan
void FireExtinguisher::fanOff(void) {
    digitalWrite(fanPin, LOW);
}
