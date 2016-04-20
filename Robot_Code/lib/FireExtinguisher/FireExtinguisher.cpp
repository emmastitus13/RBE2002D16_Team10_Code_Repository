/* Fire Extinguisher using a propeller on a pan and tilt bracket with an infrared flame sensor
 * Uses 1 fan motor, 1 micro servo, and 1 infrared flame sensor
 * Pins
 * FAN_PIN - 1 Digital I/O
 * TILT_SERVO - 1 PWM Port
 * FLAME_SENSOR - 1 Analog Input, 1 Digital Input
 *
 * Created on Apr 12, 2016 by Ben Titus
 * Last edit made Apr 17, 2016 by Ben Titus
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


//sets the min and max values for the tilt servo and attaches the servo
void FireExtinguisher::setServo(uint8_t min, uint8_t max) {
    tiltServo.attach(servoPin);
    servoMin = min;
    servoMax = max;
    servoPos = 0;
}


//turns on the fan and reads the flame sensor until the flame is extinguished
//shouldn't run if no flame is sensed
void FireExtinguisher::extinguishFire(void) {
    int val = readFlameSense();
    while (val < 950) {
        fanOn();
        //delay(2000);
        val = readFlameSense();
    }
    fanOff();
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
//could be turned into a "getZ" function later using servoPos and the frontUS sensor
int FireExtinguisher::getDistance(void) {
    int val = analogRead(flameSensePinA);
    return val / flameSensorConstant;
}


//tilts the servo to an angle, causing the fan array to tilt up and down
//won't write higher or lower than the max/min values of the servo
void FireExtinguisher::servoTilt(int tiltTo) {
    if (tiltTo > servoMax) {
        tiltTo = servoMax;
    }
    if (tiltTo < servoMin) {
        tiltTo = servoMin;
    }
    tiltServo.write(tiltTo);
    servoPos = tiltTo;
}


//moves the servo up and down to find the flame
int FireExtinguisher::findFlame(void) {
    servoTilt(servoMin); //reset the tilt servo to minimum position

    int tolerance = 40;
    int pastFlameVal, flameVal;
    pastFlameVal = 1024; //set the past value to maximum
    flameVal = readFlameSense(); //get the current flame sensor value

    for (int i = servoMax; i > servoMin; i--) {
      Serial.print(flameVal);
      Serial.print(" ");
      Serial.print(pastFlameVal);
      Serial.print(" ");
      Serial.println(i);
        if (flameVal < tolerance) {
          Serial.println("Sup");
            return i;
        } else {
            pastFlameVal = flameVal;
            servoTilt(i);
            flameVal = readFlameSense();
            Serial.println("Dawg");
        }
        delay(100);
    }
}


//turns on the fan
void FireExtinguisher::fanOn(void) {
    if (servoPos < servoMin) {
        //this breaks the fans
    } else {
        digitalWrite(fanPin, HIGH);
    }
}


//turns off the fan
void FireExtinguisher::fanOff(void) {
    digitalWrite(fanPin, LOW);
}
