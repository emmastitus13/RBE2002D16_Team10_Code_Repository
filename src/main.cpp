/* RBE2002D16 Final Project Code
 *
 * Will not wall follow since the flame sensor cannot rotate independently of the robot
 * Will find the candle by exploratory methods :D
 *
 *
 *
 * Created on Apr 12. 2016 by Ben Titus
 * Last edit made Apr 27, 2016 by Ben Titus
 */
#include <Arduino.h>
#include <Servo.h>
#include <Math.h>
#include "TimerOne.h"
#include "LiquidCrystal.h"
#include "Gyro.h"
#include "definitions.h"
#include "Robot.h"
#include "Turn.h"
#include "DriveTrain.h"
#include "FireExtinguisher.h"
#include "hBridgeMotorDriver.h"
#include "NewPing.h"
#include "DebugLED.h"

//funciton prototypes
void findAndExtinguishCandle(void);
void lEncoderISR(void);
void rEncoderISR(void);
void timer1ISR(void);
void frontBumpISR(void);

bool turnLeft90(void);
bool turnRight90(void);

unsigned long readUS(NewPing us);


bool candleFind(void);
uint8_t candleTestHigh(void);
void candleSlowSweep(void);
void candleSlightSweep(void);





//object declarations
Gyro gyro;
NewPing leftUS(LEFT_US_TP, LEFT_US_EP, MAX_DISTANCE);
NewPing rightUS(RIGHT_US_TP, RIGHT_US_EP, MAX_DISTANCE);
NewPing frontUS(FORWARD_US_TP, FORWARD_US_EP, MAX_DISTANCE);
DriveTrain robotDrive(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, MAX_MOTOR_SPEED);
FireExtinguisher fireExtinguisher(FAN_PIN, FLAME_SENSE_PINA, FLAME_SENSE_PIND, TILT_SERVO_PIN, FLAME_SENSOR_CONSTANT);
LiquidCrystal LCD(RS_PIN, EN_PIN, DB1_PIN, DB2_PIN, DB3_PIN, DB4_PIN);
DebugLED orange(ORANGE_LED_PIN);
DebugLED blue(BLUE_LED_PIN);
Turn turn(robotDrive);
Robot robot(LCD, fireExtinguisher, robotDrive, leftUS, rightUS,frontUS,orange,blue,gyro);



//test turns on Serial and debugLEDs
bool test = true;
uint8_t demoState = FULL_DEMO;


//general variables
bool go = false;
bool upDown;
bool gyroGood = false;
uint8_t ii;
int tempServoMin;
int servoIndex;
bool drawn = false;
uint8_t wallCount = 0;


//states
volatile uint8_t botState = STOP;
uint8_t wallState = WALL_TEST;
uint8_t candleState = CANDLE_FIND;
uint8_t turnState = IMU_TURN;
uint8_t mazeState = MAZE_TEST;
uint8_t aroundState = LEAVE_WALL;
uint8_t sweepState = 0;
uint8_t slowSweepState = 1;
uint8_t slightSweepState = 0;
uint8_t wallSweepState = SWEEP_FORWARDS;


//encoder values
volatile unsigned long lEncode = 0;
volatile unsigned long rEncode = 0;
unsigned long curRTicks = 0;
unsigned long curLTicks = 0;
unsigned int tickRDiff = 0;
unsigned int tickLDiff = 0;
int pastlEnc = 0;
int pastrEnc = 0;

//timer values
volatile unsigned int timer1cnt = 0;
volatile unsigned int timer = 0;

//motor values
unsigned long currentL = 0, currentR = 0;
int servoMaximum, servoMinimum, servoPosition;

//ultrasonic sensor values
unsigned long lUSVal, rUSVal, frUSVal;

//driving variables
volatile bool frBumpPush = false;
uint8_t baseDrive = 220;
uint8_t driveL = baseDrive;
uint8_t driveR = baseDrive - 3;
bool lastWall;

//movement variables
uint8_t globi = 0;
int buffTicks= 0;
float tempAngle = 0.0;
float angAttempt = 0.0;
Movement movBuf = {globi, buffTicks, tempAngle};

//gyro variables
//gyro code taken from example provided by Joe St. Germain
//Code was tweaked to be more consistant for accurate turns
float G_Dt=0.005;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long gyroTimer=0;   //general purpose timer
long gyroTimer1=0;

float G_gain=.008699; // gyros gain factor for 250deg/se
float gyro_z; //gyro x val
float gyro_zold; //gyro cummulative z value
float gerrz; // Gyro 7 error
float minErr = 0.005;


//candle positions
float xPos = 0; //x position of the candle
float yPos = 0; //y position of the candle
float zPos = 0; //z position of the candle


//arrays
NewPing USSensors[3] ={leftUS, frontUS, rightUS};
unsigned long USVals[3] = {lUSVal, rUSVal, frUSVal};
Movement movements[ARRAY_LENGTH];
float xMov[ARRAY_LENGTH], yMov[ARRAY_LENGTH];
int flameVals[90];


/*************************************************************************************************************************/
void setup() {
    blue.debugLEDOFF();
    orange.debugLEDOFF();

    LCD.begin(16,2);
    LCD.setCursor(0,1);
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
    curRTicks = 0;
    curLTicks = 0;
    tickRDiff = 0;
    tickLDiff = 0;
    tempAngle = 0;

    robotDrive.botStop();
    fireExtinguisher.fanOff();
    go = true;
}

/*************************************************************************************************************************/
void loop() {
    findAndExtinguishCandle();
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
        orange.debugLEDON();
        blue.debugLEDON();
        LCD.setCursor(0,0);
        LCD.print("ERR");
    }
}


/*************************************************************************************************************************/
//Main program
void findAndExtinguishCandle(void) {
    switch (botState) {
        case STOP:
            if (go) {
                botState = NAVIGATE_MAZE;
            }
            break;

        case NAVIGATE_MAZE:
            if (robot.mazeSearch()) {
                botState = FIND_CANDLE;
            }
            break;

        case FIND_CANDLE:
            if (robot.candleFind()) {
                botState = CALCULATE_VALUES;
            }
            break;

        case CALCULATE_VALUES:
            robot.updatePosition(xPos, yPos, zPos);
            botState = RETURN_HOME;
            break;

        case RETURN_HOME:
            break;
    }
}




/*************************************************************************************************************************/
//reads the values of a US sensor
unsigned long readUS(NewPing us) {
    return us.ping_cm();
}

//turns the robot 90 degrees to the left
bool turnLeft90(void) {
    switch (turnState) {
        case ENCODER_TURN:
            tickRDiff = (rEncode - curRTicks);
            tickLDiff = (lEncode - curLTicks);

            if ((tickRDiff < tickPer90) || (tickLDiff < tickPer90)) {
                robotDrive.botDrive(153, -153);
                return false;
            }
            robotDrive.botStop();
            movBuf.angle += 90;
            robot.logMove(movBuf);
            //reset encoder values
            curLTicks = lEncode;
            curRTicks = rEncode;
            tickLDiff = 0;
            tickRDiff = 0;
            return true;
            break;

        case IMU_TURN:
        gyro.gyroRead();
            if (gyro_z < IMU_TURN_90_LEFT) {
                robotDrive.botDrive(153, -153);
                return false;
            }
            robotDrive.botStop();
            movBuf.angle += 90;
            robot.logMove(movBuf);
            LCD.setCursor(0,0);
            LCD.print(gyro_z);
            //reset the gyro
            curLTicks = lEncode;
            curRTicks = rEncode;
            tickLDiff = 0;
            tickRDiff = 0;
            gyroGood = false;
            gyro_z = 0;
            gyro_zold = 0;
            return true;
            break;

        default:
            return false;
    }
}


//turns the robot 90 degrees to the right
bool turnRight90(void) {
    switch (turnState) {
        case ENCODER_TURN:
            tickRDiff = (rEncode - curRTicks);
            tickLDiff = (lEncode - curLTicks);

            if ((tickRDiff < tickPer90) || (tickLDiff < tickPer90)) {
                robotDrive.botDrive(-153, 153);
                return false;
            }
            robotDrive.botStop();
            movBuf.angle -= 90;
            robot.logMove(movBuf);
            //reset encoder values
            curLTicks = lEncode;
            curRTicks = rEncode;
            tickLDiff = 0;
            tickRDiff = 0;
            return true;
            break;

        case IMU_TURN:
            gyro.gyroRead();
            if (gyro_z > IMU_TURN_90_RIGHT) {
                robotDrive.botDrive(-153, 153);
                return false;
            }
            robotDrive.botStop();
            movBuf.angle -= 90;
            robot.logMove(movBuf);
            LCD.setCursor(0,0);
            LCD.print(gyro_z);
            //reset the gyro
            curLTicks = lEncode;
            curRTicks = rEncode;
            tickLDiff = 0;
            tickRDiff = 0;
            gyroGood = false;
            gyro_z = 0;
            gyro_zold = 0;
            return true;
            break;

        default:
            return false;
    }
}


/*************************************************************************************************************************/
