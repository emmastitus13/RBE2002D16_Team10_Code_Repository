// Two wheel drive train with two H-bridge driven wheels
/* One wheel turns opposite the other
 *
 * Does not contain any encoder information. That is handled in main.cpp
 *
 * Pins
 * Left motor - 2 PWM pins
 * Right motor - 2 pwm pins
 *
 * Created Apr 12, 2016 by Ben Titus
 * Last edit made Apr 17, 2016 by Ben Titus
 */

#ifndef DriveTrain_h
#define DriveTrain_h

#include <Arduino.h>
#include "hBridgeMotorDriver.h"

class DriveTrain {
public:
    DriveTrain(uint8_t lMoto1, uint8_t lMoto2, uint8_t rMoto1, uint8_t rMoto2, uint8_t maxSPD);
    void botTurnLeft(void); //rotates left about the turning center at full speed
    void botTurnRight(void); // rotates right about the turning center at full speed
    void botTurnLeft(uint8_t spd); //rotates left about the turning center at a certain speed
    void botTurnRight(uint8_t spd); //rotates right about the turning center at a certain speed
    void botDrive(int lspd, int rspd); //drives each wheel at specified speeds. Can handle forward and backwards
    void botStop(void); //stops the robot
    void attachMotors(void); //initializes the motors
    uint8_t maxSpeed; //maximum speed the motor will go
    void frontBumperPush(void); //disables motor functionality

private:
    bool fronBump = false;
    uint8_t lMotorPin1, lMotorPin2, //PWM pins to control left motor
            rMotorPin1, rMotorPin2; //PWM pins to control right motor
    Type2_Motor leftMotor, rightMotor; //hBridgeMotorLibrary Type2_Motors
};

#endif
