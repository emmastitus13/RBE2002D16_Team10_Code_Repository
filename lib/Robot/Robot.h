#ifndef ROBOT_H
#define ROBOT_H

#include "LiquidCrystal.h"
#include "FireExtinguisher.h"
#include "DriveTrain.h"
#include <Arduino.h>

extern uint8_t globi;
extern unsigned long USVals[3];
extern uint8_t mazeState;
extern flaot zPos;

class Robot{
  public:
    Robot(LiquidCrystal& lcd, FireExtinguisher& fr, DriveTrain& rb);
    void updatePosition(int newX, int newY, int newZ); //check
    void logMove(Movement mov); //check
    void readAllUS(void); //check
    void printAllUS(void) ; //check
    void candleZ(void); //check
    uint8_t candleTest(void);

  private:
  	LiquidCrystal& LCD;
  	FireExtinguisher& fireExtinguisher;
  	DriveTrain& robotDrive;
};

#endif
