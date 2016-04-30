/* Motor Driver Library for driving an H-bridge controlled motor
 * Assumes the following connections
 * Type1
 * enPin  - PWM Pin
 * in1Pin - Digital IO
 * in2Pin - Digital IO
 * Type2
 * in1Pin - PWM Pin
 * in2Pin - PWM Pin
 *
 * Created by Ben Titus, April 5, 2016
 * Last edit made by Ben Titus, April 17, 2016
 */


#ifndef hBridgeMotorDriver_h
#define hBridgeMotorDriver_h

#include <Arduino.h>

class Type1_Motor {
public:
    Type1_Motor(); //general constructor
    Type1_Motor(uint8_t en, uint8_t in1, uint8_t in2); //specific constructor
    void driveForward(uint8_t spd); //drives the motor forward
    void driveBackward(uint8_t spd); //drives the motor backward
    void drive(int spd); //drives the motor forward or backward
    void brake(void); //stops the motor
    void setPins(uint8_t en, uint8_t in1, uint8_t in2); //initializes the motor

private:
    uint8_t enPin, in1Pin, in2Pin; //pins for the motor
};


class Type2_Motor {
public:
    Type2_Motor(); //general constructor
    Type2_Motor(uint8_t in1, uint8_t in2); //specific constructor
    void driveForward(uint8_t spd); //drives the motor forward
    void driveBackward(uint8_t spd); //drives the motor backward
    void drive(int spd); //drives the motor forward or backward
    void brake(void); //stops the motor
    void setPins(uint8_t in1, uint8_t in2); //initializes the motor

private:
    uint8_t in1Pin, in2Pin;
};


#endif
