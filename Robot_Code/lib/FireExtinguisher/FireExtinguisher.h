/* Fire Extinguisher using a propeller on a pan and tilt bracket with an infrared flame sensor
 * Uses 1 fan motor, 1 micro servo, and 1 infrared flame sensor
 * Pins
 * FAN_PIN - 1 Digital I/O
 * TILT_SERVO - 1 PWM Port
 * FLAME_SENSOR - 1 Analog Input, 1 Digital Input
 *
 * Created on Apr 12. 2016 by Ben Titus
 * Last edit made Apr 17, 2016 by Ben Titus
 */

#ifndef FireExtinguish_h
#define FireExtinguish_h

#include <Arduino.h>
#include <Servo.h>

class FireExtinguisher {
    public:
        FireExtinguisher(uint8_t fan, uint8_t flameSenseA, uint8_t flameSenseD, uint8_t servo, int flameConst);

        void setServo(uint8_t min, uint8_t max); //attaches pins and sets the minimum and maximum values for the servo to turn to
        void extinguishFire(void); //if flame, turns on fan until flame gone
        int readFlameSense(void); //returns the analog value on the flame sensor
        int readFlameSenseDig(void); //returns the digital value on the flame sensor
        int getDistance(void); //probably won't be used, but converts analog value/servo position into flame distance
        void servoTilt(int tiltTo); //moves the servo to the specified position
        int findFlame(void); //scans up until flame is found then returns servoPos
        void fanOn(void); //turns the fan on
        void fanOff(void); //turns the fan off
        uint8_t servoPos, //current position of the servo
                servoMin, //minimum value of the servo (fans pointing almost flat)
                servoMax; //maximum value of the servo (fans pointing almost up)

    private:
        uint8_t fanPin, flameSensePinA, flameSensePinD, servoPin;
        int flameSensorConstant, //probably not useful
            flameVal, //stores the analog value of the flame sensor
            pastFlameVal; //stores the previous analog value of the flame sensor
        Servo tiltServo;
};

#endif
