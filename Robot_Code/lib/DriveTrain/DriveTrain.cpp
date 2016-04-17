// Two wheel drive train with two H-bridge driven wheels
/* One wheel turns opposite the other
 * Pins
 * Left motor - 2 PWM pins
 * Right motor - 2 pwm pins
 *
 * Created Apr 12, 2016 by Ben Titus
 * Last edit made Apr 12, 2016 by Ben Titus
 */

#include "DriveTrain.h"


DriveTrain::DriveTrain(uint8_t lMoto1, uint8_t lMoto2, uint8_t rMoto1, uint8_t rMoto2, uint8_t maxSPD) {
    lMotorPin1 = lMoto1;
    lMotorPin2 = lMoto2;
    rMotorPin1 = rMoto1;
    rMotorPin2 = rMoto2;
    maxSpeed = maxSPD;
}

void DriveTrain::attachMotors(void) {
    //connect the wires backwards on one motor to make them spin the same direction
    leftMotor.setPins(lMotorPin1, lMotorPin2);
    rightMotor.setPins(rMotorPin2, rMotorPin1);
}


//writes to both motors to make the robot turn left at full speed
void DriveTrain::botTurnLeft(void) {
    rightMotor.driveBackward(maxSpeed);
    leftMotor.driveForward(maxSpeed);
}


//writes to both motors to make the robot turn right at full speed
void DriveTrain::botTurnRight(void) {
    leftMotor.driveBackward(maxSpeed);
    rightMotor.driveForward(maxSpeed);
}


//writes to both motors to make the robot turn left at a certain speed
void DriveTrain::botTurnLeft(uint8_t spd) {
    rightMotor.driveBackward(spd);
    leftMotor.driveForward(spd);
}


//writes to both motors to make the robot turn right at a certain speed
void DriveTrain::botTurnRight(uint8_t spd) {
    leftMotor.driveBackward(spd);
    rightMotor.driveForward(spd);
}


//writes to both motors to make them rotate at certain speeds
void DriveTrain::botDrive(int lspd, int rspd) {
    leftMotor.drive(lspd);
    rightMotor.drive(rspd);
}


//writes both motors low to brake and stop the robot
void DriveTrain::botStop(void) {
    leftMotor.brake();
    rightMotor.brake();
}
