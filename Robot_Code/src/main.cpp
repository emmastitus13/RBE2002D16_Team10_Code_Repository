/* RBE2002D16 Final Project Code
 *
 * Will not wall follow since the flame sensor cannot rotate independently of the robot
 * Will find the candle by exploratory methods :D
 *
 *
 *
 * Created on Apr 12. 2016 by Ben Titus
 * Last edit made Apr 23, 2016 by Ben Titus
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


//test turns on Serial and debugLEDs
bool test = true;


//funciton prototypes
void logMove(void);
unsigned long readUS(NewPing us);
void readAllUS(void);
void findAndExtinguishCandle(void);
void lEncoderISR(void);
void rEncoderISR(void);
void timer1ISR(void);
void frontBumpISR(void);
bool oneRotato(void);
void driveStraight(void);
uint8_t wallTest(void);
void wallNav(void);
bool turnLeft90(void);
bool turnRight90(void);
void candleFind(void);
uint8_t candleTest(void);


//object declarations
NewPing leftUS(LEFT_US_TP, LEFT_US_EP, MAX_DISTANCE);
NewPing rightUS(RIGHT_US_TP, RIGHT_US_EP, MAX_DISTANCE);
NewPing frontUS(FORWARD_US_TP, FORWARD_US_EP, MAX_DISTANCE);
DriveTrain robotDrive(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, MAX_MOTOR_SPEED);
FireExtinguisher fireExtinguisher(FAN_PIN, FLAME_SENSE_PINA, FLAME_SENSE_PIND, TILT_SERVO_PIN, FLAME_SENSOR_CONSTANT);
LiquidCrystal LCD(RS_PIN, EN_PIN, DB1_PIN, DB2_PIN, DB3_PIN, DB4_PIN);
DebugLED orange(ORANGE_LED_PIN);
DebugLED blue(BLUE_LED_PIN);


//states
uint8_t wallState = WALL_TEST;
volatile uint8_t botState = STOP;
uint8_t candleState = CANDLE_FIND;


//encoder values
volatile unsigned long lEncode = 0;
volatile unsigned long rEncode = 0;
unsigned long curRTicks = 0;
unsigned long curLTicks = 0;
unsigned int tickRDiff = 0;
unsigned int tickLDiff = 0;

//timer values
volatile unsigned int timer1cnt = 0;
volatile unsigned int timer = 0;

//motor values
unsigned long currentL = 0, currentR = 0;
int servoMaximum, servoMinimum, servoPosition;

//ultrasonic sensor values
unsigned long lUSVal, rUSVal, frUSVal;

//general variables
bool upDown;

//
volatile bool frBumpPush = false;
int pastlEnc = 0, pastrEnc = 0;
uint8_t baseDrive = 255;
uint8_t driveL = baseDrive;
uint8_t driveR = baseDrive;
uint8_t globi = 0;
int buffTicks= 0;
float tempAngle = 0.0;
Movement movBuf = {globi, buffTicks, tempAngle};


float xPos = 0; //x position of the candle
float yPos = 0; //y position of the candle
float zPos = 0; //z position of the candle


NewPing USSensors[3] ={leftUS, frontUS, rightUS};
unsigned long USVals[3] = {lUSVal, rUSVal, frUSVal};
Movement movements[ARRAY_LENGTH];
float xMov[ARRAY_LENGTH], yMov[ARRAY_LENGTH];


/*************************************************************************************************************************/
void setup() {
    blue.debugLEDOFF();
    orange.debugLEDOFF();

    LCD.begin(16,2);
    //LCD.setCursor(0,0);
    LCD.print("LOADING");

    if (test) {
        Serial.begin(115200);
    }

    Timer1.initialize(1000); //1ms timer to time some things. open to changes
    Timer1.attachInterrupt(timer1ISR);

    LCD.print(" 1...");

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
    orange.debugLEDFlash();
    candleSweep();
    delay(250);
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
    LCD.clear();
    while (true) {
        robotDrive.botStop();
        Serial.println("ESTOP");
        orange.debugLEDFlash();
        blue.debugLEDFlash();
        LCD.setCursor(0,0);
        LCD.print("ERR");
    }
}


/*************************************************************************************************************************/
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


//moves the robot forward one wheel rotation
bool oneRotato() {
    // if (test) {
    //     Serial.print(lEncode);
    //     Serial.print(" ");
    //     Serial.println(rEncode);
    // }

    tickRDiff = (rEncode - curRTicks);
    tickLDiff = (lEncode - curLTicks);

    if((tickLDiff <= (encTicksPerWheelRev / 2)) || (tickRDiff <= (encTicksPerWheelRev / 2))){
        wallState = wallTest();
        driveStraight();
        return false;
    } else {
        //robotDrive.botStop();
        return true;
    }
}


//determines where a wall is, if there is a wall
uint8_t wallTest() {
    readAllUS();
    if (driveL < (baseDrive - 15)) {
        driveL = baseDrive - 5;
    }
    if (driveR < (baseDrive - 15)) {
        driveR = baseDrive - 5;
    }
    curLTicks = lEncode;
    curRTicks = rEncode;
    tickLDiff = 0;
    tickRDiff = 0;
    if ((USVals[2] < 10) && (USVals[2] > 0)) { //if a wall in front
        if ((USVals[0] > USVals[1]) && (USVals[1] > 0)) { //if a left wall is nearer than a right wall
                if (test) {
                    orange.debugLEDON();
                    blue.debugLEDOFF();
                    Serial.println("LEFT");
                }
                return TURN_RIGHT;
        } else { //if a right wall is nearer than a left wall or no wall is nearer
                if (test) {
                    blue.debugLEDON();
                    orange.debugLEDOFF();
                    Serial.println("RIGHT");
                }
                return TURN_LEFT;
        }
    }

    blue.debugLEDOFF();
    orange.debugLEDOFF();

    if ((USVals[0] <= 4) && (USVals[0] > 0)) { //if a left wall is near
        blue.debugLEDON();
        Serial.println("DriveLeft++");
        blue.debugLEDOFF();
        return WALL_LEFT;
    }

    if ((USVals[1] <= 4) && (USVals[1] > 0)) { //if a right wall is near
        orange.debugLEDON();
        Serial.println("DriveRight++");
        orange.debugLEDOFF();
        return WALL_RIGHT;
    }

    if ((USVals[0] <= 10) || (USVals[1] <= 10)) {

        if ((USVals[0] <= 7) && (USVals[0] > 0)) { //if a left wall is near
            blue.debugLEDON();
            Serial.println("DriveLeft++");
            blue.debugLEDOFF();
            return NO_WALLS_LEFT;
        }

        if ((USVals[1] <= 7) && (USVals[1] > 0)) { //if a right wall is near
            orange.debugLEDON();
            Serial.println("DriveRight++");
            orange.debugLEDOFF();
            return NO_WALLS_RIGHT;
        }
        return FORWARD;
    }

    if ((USVals[0] > 10) && (USVals[1] > 10) && (USVals[2] > 10)) {
        return NO_WALLS;
    }
}


//wall following function
void wallNav() {
    Serial.print("tickRDiff: ");
    Serial.println(tickRDiff);
    switch(wallState) {
        case WALL_TEST:
            wallState = wallTest();
            Serial.println(curRTicks);
            break;

        case TURN_RIGHT:
            if (turnRight90()) {
                wallState = WALL_TEST;
            } else {}
            break;

        case TURN_LEFT:
            if (turnLeft90()) {
                wallState = WALL_TEST;
            } else {}
            break;

        case WALL_RIGHT:
            driveL--;
            wallState = FORWARD;
            break;

        case WALL_LEFT:
            driveR--;
            wallState = FORWARD;
            break;

        case FORWARD:
            if (oneRotato()) {
                wallState = WALL_TEST;
            } else {}
            break;

        case NO_WALLS:
            //congrats
            break;

        case NO_WALLS_RIGHT:
            driveL++;
            wallState = FORWARD;
            break;

        case NO_WALLS_LEFT:
            driveR++;
            wallState = FORWARD;
            break;
    }
}


//turns the robot 90 degrees to the left
bool turnLeft90(void) {

    tickRDiff = (rEncode - curRTicks);
    tickLDiff = (lEncode - curLTicks);

    if ((tickRDiff <= tickPer90) || (tickLDiff <= tickPer90)) {
        robotDrive.botTurnLeft();
        return false;
    } else {
        robotDrive.botStop();
        return true;
    }
}


//turns the robot 90 degrees to the right
bool turnRight90(void) {

    tickRDiff = (rEncode - curRTicks);
    tickLDiff = (lEncode - curLTicks);

    if ((tickRDiff <= tickPer90) || (tickLDiff <= tickPer90)) {
        robotDrive.botTurnRight();
        return false;
    } else {
        robotDrive.botStop();
        return true;
    }
}


//calculates the position of the candle then stores it in xPos, yPos, and zPos
//this function will be called after extinguishing the candle
void calcDist(Movement mov) {
    xPos = 0;
    yPos = 0;

    float mag = ((mov.encTicks / encTicksPerWheelRev) * wheelCirc) / 10.0;
    xMov[mov.index] = mag * cos(mov.angle);
    yMov[mov.index] = mag * sin(mov.angle);

    for (int i = 0; i < ARRAY_LENGTH; i++) {
        xPos += xMov[i];
        yPos += yMov[i];
    }

    //use the servoPos to calculate the z position of the candle
}


//sweeps back and forth to find the candle
void candleFind(void) {
    fireExtinguisher.servoTilt(70);
    bool dig = fireExtinguisher.readFlameSenseDig();
    uint8_t an = fireExtinguisher.readFlameSenseDig();
    Serial.print("DIG: ");
    Serial.print(dig);
    Serial.print("ANALOG: ");
    Serial.println(an);

    switch(candleState) {
        case FIND_CANDLE:

            candleTest();
            break;

        case CANDLE_FOUND:
            break;
    }
}


//sweeps back and forth to search for the candle
void candleTest(void) {
    bool dig = fireExtinguisher.readGlameSenseDig();
    uint8_t an = fireExtinguisher.readFlameSense();

    if (dig) {
        candleState = CANDLE_FOUND;
    }
    
}
