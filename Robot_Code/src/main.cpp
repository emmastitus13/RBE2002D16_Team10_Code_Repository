/* RBE2002D16 Final Project Code
 *
 * Will not wall follow since the flame sensor cannot rotate independently of the robot
 * Will find the candle by exploratory methods :D
 *
 *
 *
 * Created on Apr 12. 2016 by Ben Titus
 * Last edit made Apr 22, 2016 by Ben Titus
 */

#include <Arduino.h>
#include <Servo.h>
#include "LiquidCrystal.h"
#include "TimerOne.h"
#include <Wire.h>
#include "L3G.h"
#include "definitions.cpp"
#include "gyro.cpp"
#include "DriveTrain.h"
#include "FireExtinguisher.h"
#include "hBridgeMotorDriver.h"
#include "NewPing.h"
#include "DebugLED.h"

bool test = true;

void logMove(void);
unsigned long readUS(NewPing us);
void readAllUS(void);
void findAndExtinguishCandle(void);
void lEncoderISR(void);
void rEncoderISR(void);
void timer1ISR(void);
void frontBumpISR(void);
void oneRotato(void);
void driveStraight(void);
uint8_t testWalls(void);
void wallNav(void);
void turnLeft90(void);
void turnRight90(void);


NewPing leftUS(LEFT_US_TP, LEFT_US_EP, MAX_DISTANCE);
NewPing rightUS(RIGHT_US_TP, RIGHT_US_EP, MAX_DISTANCE);
NewPing frontUS(FORWARD_US_TP, FORWARD_US_EP, MAX_DISTANCE);
DriveTrain robotDrive(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, MAX_MOTOR_SPEED);
FireExtinguisher fireExtinguisher(FAN_PIN, FLAME_SENSE_PINA, FLAME_SENSE_PIND, TILT_SERVO_PIN, FLAME_SENSOR_CONSTANT);
LiquidCrystal LCD(RS_PIN, EN_PIN, DB1_PIN, DB2_PIN, DB3_PIN, DB4_PIN);
DebugLED orange(ORANGE_LED_PIN);
DebugLED blue(BLUE_LED_PIN);

const int fullEncTicksPerWheelRev = 3575;
const int encTicksPerWheelRev = 1788;
volatile unsigned char botState = STOP;
volatile unsigned long lEncode = 0;
volatile unsigned long rEncode = 0;
volatile unsigned int timer1cnt = 0;
volatile unsigned int timer = 0;
unsigned long currentL = 0, currentR = 0;
unsigned long lUSVal, rUSVal, frUSVal;
int servoMaximum, servoMinimum, servoPosition;
bool upDown;
bool frBumpPush = false;
int pastlEnc = 0, pastrEnc = 0;
uint8_t baseDrive = 255;
uint8_t driveL = baseDrive;
uint8_t driveR = baseDrive;
Movement movBuf;

NewPing USSensors[3] ={leftUS, frontUS, rightUS};
unsigned long USVals[3] = {lUSVal, rUSVal, frUSVal};
Movement movements[64];

/*************************************************************************************************************************/
void setup() {
    if (test) {
        Serial.begin(115200);
    }

    Timer1.initialize(1000); //10ms timer to time some things. open to changes
    Timer1.attachInterrupt(timer1ISR);

    pinMode(L_ENCODER_PIN, INPUT_PULLUP);
    pinMode(R_ENCODER_PIN, INPUT_PULLUP);
    pinMode(FRONT_BUMPER, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN), lEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN), rEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FRONT_BUMPER), frontBumpISR, FALLING);

    fireExtinguisher.setServo(30, 120);
    robotDrive.attachMotors();

    servoMaximum = fireExtinguisher.servoMax;
    servoMinimum = fireExtinguisher.servoMin;
    servoPosition = fireExtinguisher.servoPos;
    upDown = true;

    robotDrive.botStop();
    fireExtinguisher.fanOff();
    timer = millis();
}

/*************************************************************************************************************************/
void loop() {
    testWalls(); //searches for a wall
    wallNav(); //currently blocking :(
 }

/*************************************************************************************************************************/
//ISR for the 10ms timer
void timer1ISR(void) {
    timer1cnt++;
    timer++;
}

//ISR for the left wheel encoder
void lEncoderISR(void) {
    lEncode++;
}

//ISR for the right wheel encoder
void rEncoderISR(void) {
    rEncode++;
}

//ISR for the front bumper
void frontBumpISR(void) {
    while (true) {
        robotDrive.botStop();
        Serial.println("ESTOP");
        delay(1);
    }
}

//Main program
void findAndExtinguishCandle(void) {
    switch (botState) {
        case STOP:
            break;

        case FIND_CANDLE:
            break;

        case EXTINGUISH_FIRE:
            break;

        case RETURN_HOME:
            break;
    }
}



//updates the LCD to display the current position
void updatePosition(int newX, int newY, int newZ) {
    char pos[10];
    sprintf(pos, "%2d, %2d, %2d", newX, newY, newZ);
    LCD.setCursor(0,0);
    LCD.print("Flame Position:");
    LCD.setCursor(0,1);
    LCD.print(pos);
}

//records a movement
//movement consists of an angle, a number of ticks
//movements recorded in array of movements
void logMove() {
    movements[globi] = movBuf;
    globi++;
}


//reads the values of a US sensor
unsigned long readUS(NewPing us) {
    return us.ping_cm();
}


//reads all three US sensors
void readAllUS(void) {
        USVals[0] = leftUS.ping_cm();
        delay(25);

        USVals[1] = rightUS.ping_cm();
        delay(25);

        USVals[2] = frontUS.ping_cm();
        delay(25);
}


//drives the robot straight according to the encoders
void driveStraight() {
    robotDrive.botDrive(driveL, driveR);
    if ((timer1cnt % 500) == 1) {
        int diffL = lEncode - pastlEnc;
        int diffR = rEncode - pastrEnc;
        if (diffL - diffR > 0) {
            driveL--;
        }
        if (diffR - diffL > 0) {
            driveL++;
        }
        if (test) {
            Serial.print("Leftt / sec: ");
            Serial.print(diffL * 2);
            Serial.print(" Right / sec: ");
            Serial.println(diffR * 2);
        }
        pastrEnc = rEncode;
        pastlEnc = lEncode;
        if (driveL < (baseDrive - 15)) {
            driveL = baseDrive;
        }
        if (driveR < (baseDrive - 15)) {
            driveR = baseDrive;
        }
    }
}

void oneRotato(){
    if (test) {
        Serial.print(lEncode);
        Serial.print(" ");
        Serial.println(rEncode);
    }

    if((lEncode >= encTicksPerWheelRev) || (rEncode >= encTicksPerWheelRev)){
      robotDrive.botStop();
    } else {
      driveStraight();
    }
}

//determines where a wall is, if there is a wall
uint8_t testWalls() {
    readAllUS();
    if ((USVals[2] < 10) && (USVals[2] > 0)) { //if a wall in front
        if ((USVals[0] > USVals[1]) && (USVals[1] > 0)) { //if a left wall is nearer than a right wall
                if (test) {
                    orange.debugLEDON();
                    Serial.println("LEFT");
                }
                return TURN_RIGHT;
        } else { //if a right wall is nearer than a left wall or no wall is nearer
                if (test) {
                    blue.debugLEDON();
                    Serial.println("RIGHT");
                }
                return TURN_LEFT;
        }
    }

    if ((USVals[0] < 10) && (USVals[0] > 0)) { //if a left wall is near
        //Serial.println("DriveLeft++");
        return WALL_LEFT;
    }

    if ((USVals[1] < 10) && (USVals[1] > 0)) { //if a right wall is near
        //Serial.println("DriveRight++");
        return WALL_RIGHT;
    }

    if ((USVals[0] > 10) || (USVals[1] > 10)) {
        return FORWARD;
    }

    blue.debugLEDOFF();
    orange.debugLEDOFF();
}

void wallNav() {
    switch(wallState) {
        case TEST:
            testWalls();
            break;

        case TURN_RIGHT:
            turnRight90();
            wallState = TEST;
            break;

        case TURN_LEFT:
            turnLeft90();
            wallState = TEST;
            break;

        case FORWARD:
            oneRotato();
            break;

    }
}

//returns the distance of a movement

//turns the robot 90 degrees based on encoders

//If you think of anything else, add it here
