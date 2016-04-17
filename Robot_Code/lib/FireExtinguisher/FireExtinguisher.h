/* Fire Extinguisher using a propeller on a pan and tilt bracket with an infrared flame sensor
 * Uses 1 fan motor, 1 micro servo, and 1 infrared flame sensor
 * Pins
 * FAN_PIN - 1 Digital I/O
 * TILT_SERVO - 1 PWM Port
 * FLAME_SENSOR - 1 Analog Input, 1 Digital Input
 *
 * Created on Apr 12. 2016 by Ben Titus
 * Last edit made Apr 16, 2016 by Ben Titus
 */

#ifndef FireExtinguish_h
#define FireExtinguish_h

#include <Arduino.h>
#include <Servo.h>

class FireExtinguisher {
    public:
        FireExtinguisher(uint8_t fan, uint8_t flameSenseA, uint8_t flameSenseD, uint8_t servo, int flameConst);
        void setServo(uint8_t min, uint8_t max);
        void extinguishFire(void);
        int readFlameSense(void);
        int readFlameSenseDig(void);
        int getDistance(void);
        void servoTilt(int tiltTo);
        int findFlame(void);
        void fanOn(void);
        void fanOff(void);
        uint8_t servoPos, servoMin, servoMax;

    private:
        uint8_t fanPin, flameSensePinA, flameSensePinD, servoPin;
        int flameSensorConstant, flameVal, pastFlameVal;
        Servo tiltServo;
};

#endif
