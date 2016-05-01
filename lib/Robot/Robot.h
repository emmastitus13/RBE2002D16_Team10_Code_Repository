#ifndef ROBOT_H
#define ROBOT_H

#include "LiquidCrystal.h"
#include "FireExtinguisher.h"
#include "DriveTrain.h"
#include <Arduino.h>
#include "NewPing.h"
#include "DebugLED.h"
#include "Gyro.h"
#include "definitions.h"


struct movement{
    uint8_t index;
    int encTicks;
    float angle;
} ;
typedef struct movement Movement;

extern const unsigned int encTicksPerSixthWheelRev;
extern const int tickPer5Deg;
extern uint8_t globi;
extern unsigned long USVals[3];
extern uint8_t mazeState;
extern float xPos; //x position of the candle
extern float yPos; //y position of the candle
extern float zPos;
extern volatile uint8_t botState;
extern unsigned long curRTicks;
extern unsigned long curLTicks;
extern unsigned int tickRDiff;
extern unsigned int tickLDiff;
extern volatile unsigned long lEncode;
extern volatile unsigned long rEncode;
extern uint8_t baseDrive;
extern uint8_t driveL;
extern uint8_t driveR;
extern volatile unsigned int timer1cnt;
extern Movement movBuf;
extern int pastlEnc;
extern int pastrEnc;
extern uint8_t sweepState;
extern uint8_t slowSweepState;
extern uint8_t slightSweepState;
extern uint8_t wallSweepState;
extern uint8_t turnState;
extern float tempAngle;
extern float angAttempt;
extern Movement movements[ARRAY_LENGTH];
extern uint8_t wallState;
extern uint8_t wallCount;
extern bool lastWall;
extern uint8_t aroundState;
extern Movement movements[ARRAY_LENGTH];
extern float xMov[ARRAY_LENGTH], yMov[ARRAY_LENGTH];
extern int flameVals[90];
extern uint8_t candleState;
extern int tempServoMin, servoMaximum, servoMinimum, servoPosition, servoIndex;
extern bool drawn;


class Robot{
  public:
    Robot(LiquidCrystal& lcd, FireExtinguisher& fr, DriveTrain& rb, NewPing& lus, NewPing& rus, NewPing& fus, DebugLED& orange, DebugLED& blue, Gyro& gyro);
    void updatePosition(float newX, float newY, float newZ); //check
    void updateFlameState(uint8_t state); //check
    void readAllUS(void); //check
    void printAllUS(void) ; //check
    void candleZ(void); //check
    uint8_t candleTest(void);
    uint8_t mazeWallTest(void);
    bool mazeSearch(void);
    bool rotato(uint8_t sixths);
    void driveStraight();
    bool sweep();
    bool turn5DegLeft(uint8_t deg5);
    bool turn5DegRight(uint8_t deg5);
    void logMove(Movement mov);
    bool wallNav();
    bool wallSweep(bool dir);
    uint8_t wallTest();
    bool aroundWall();
    void calcDist(Movement mov);
    bool candleFind(void);

  private:
  	LiquidCrystal& LCD;
  	FireExtinguisher& fireExtinguisher;
  	DriveTrain& robotDrive;
    NewPing& leftUS;
    NewPing& rightUS;
    NewPing& frontUS;
    DebugLED& orange;
    DebugLED& blue;
    Gyro& gyro;
};

#endif
