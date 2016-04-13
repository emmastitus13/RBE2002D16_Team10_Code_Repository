// Two wheel drive train with two H-bridge driven wheels
/* One wheel turns opposite the other
* Pins
* Left motor - 2 PWM pins
* Right motor - 2 pwm pins
*
* Created Apr 12, 2016 by Ben Titus
* Last edit made Apr 12, 2016 by Ben Titus
*/

#ifndef DriveTrain_h
#define DriveTrain_h

#include <Arduino.h>
#include "hBridgeMotorDriver.h"

class DriveTrain {
public:
    DriveTrain(uint8_t lMoto1, uint8_t lMoto2, uint8_t rMoto1, uint8_t rMoto2, uint8_t maxSPD);
    void botTurnLeft(void);
    void botTurnRight(void);
    void botTurnLeft(uint8_t spd);
    void botTurnRight(uint8_t spd);
    void botDrive(int lspd, int rspd);
    void botStop(void);

private:
    uint8_t lMotorPin1, lMotorPin2, rMotorPin1, rMotorPin2, maxSpeed;
    Type2_Motor leftMotor, rightMotor;
};

#endif
