/* RBE2002D16 Final Project Code
 *
 * Will not wall follow since the flame sensor cannot rotate independently of the robot
 * Will find the candle by exploratory methods :D
 *
 *
 *
 * Created on Apr 12. 2016 by Ben Titus
 * Last edit made Apr 25, 2016 by Ben Titus
 */

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Math.h>
#include "TimerOne.h"
#include "LiquidCrystal.h"
#include "L3G.h"

#include "definitions.cpp"

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

bool rotato(uint8_t eighths);
bool turnLeft90(void);
bool turnRight90(void);
bool turn5DegLeft(void);
bool turn5DegRight(void);
bool turnSlightLeft(void);
bool turnSlightRight(void);
void driveStraight(void);
bool sweep(void);

unsigned long readUS(NewPing us);
void readAllUS(void);

uint8_t wallTest(void);
void wallNav(void);

void candleFind(void);
uint8_t candleTest(void);
uint8_t candleTestHigh(void);
void candleSlowSweep(void);
void candleSlightSweep(void);

void mazeSearch(void);
uint8_t mazeWallTest(void);

bool gyroSetup(L3G gyro);
void gyroRead(L3G gyro);

void logMove(void);
void candleZ(void);
void updatePosition(int newX, int newY, int newZ);


//object declarations
NewPing leftUS(LEFT_US_TP, LEFT_US_EP, MAX_DISTANCE);
NewPing rightUS(RIGHT_US_TP, RIGHT_US_EP, MAX_DISTANCE);
NewPing frontUS(FORWARD_US_TP, FORWARD_US_EP, MAX_DISTANCE);
DriveTrain robotDrive(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, MAX_MOTOR_SPEED);
FireExtinguisher fireExtinguisher(FAN_PIN, FLAME_SENSE_PINA, FLAME_SENSE_PIND, TILT_SERVO_PIN, FLAME_SENSOR_CONSTANT);
LiquidCrystal LCD(RS_PIN, EN_PIN, DB1_PIN, DB2_PIN, DB3_PIN, DB4_PIN);
DebugLED orange(ORANGE_LED_PIN);
DebugLED blue(BLUE_LED_PIN);
L3G gyro;


//test turns on Serial and debugLEDs
bool test = true;


//general variables
bool go = false;
bool upDown;
bool gyroGood = false;
uint8_t ii;
int tempServoMin;
int servoIndex;
bool drawn = false;


//states
volatile uint8_t botState = STOP;
uint8_t wallState = WALL_TEST;
uint8_t candleState = CANDLE_FIND;
uint8_t turnState = IMU_TURN;
uint8_t mazeState = MAZE_TEST;
uint8_t sweepState = 0;
uint8_t slowSweepState = 1;
uint8_t slightSweepState = 0;


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
uint8_t baseDrive = 255;
uint8_t driveL = baseDrive;
uint8_t driveR = baseDrive - 2;

//movement variables
uint8_t globi = 0;
int buffTicks= 0;
float tempAngle = 0.0;
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
    LCD.setCursor(0,0);
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

    robotDrive.botStop();
    fireExtinguisher.fanOff();
    timer = millis();
    LCD.clear();
}

/*************************************************************************************************************************/
void loop() {
    wallNav();
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
            mazeSearch();
            break;

        case FIND_CANDLE:
            candleFind();
            break;

        case CALCULATE_VALUES:
            updatePosition(xPos, yPos, zPos);
            botState = RETURN_HOME;
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


//calculates the height of the candle
void candleZ(void) {
    readAllUS();
    float angle = fireExtinguisher.servoPosToAngle() * PI / 180;
    zPos = ((USVals[2] + 5) * 2 / 3 * tan(angle)) + 19;
}


/*************************************************************************************************************************/
//reads the values of a US sensor
unsigned long readUS(NewPing us) {
    return us.ping_cm();
}


//reads all three US sensors
void readAllUS(void) {
        USVals[0] = leftUS.ping_cm();
        delay(50);

        USVals[1] = rightUS.ping_cm();
        delay(50);

        USVals[2] = frontUS.ping_cm();
        delay(50);
}


/*************************************************************************************************************************/
//drives the robot straight according to the encoders
void driveStraight() {
    robotDrive.botDrive(driveL, driveR);
    if ((timer1cnt % 50) == 1) {
        int diffL = lEncode - pastlEnc;
        int diffR = rEncode - pastrEnc;
        if (diffL - diffR > 0) {
            driveL--;
        }
        if (diffR - diffL > 0) {
            driveR--;
        }
        pastrEnc = rEncode;
        pastlEnc = lEncode;
        //if one side is too slow
        if (driveL < (baseDrive - 30)) {
            if (driveL > 30) {
                driveL = baseDrive - 15;
            } else {
                driveL = baseDrive;
            }
        }
        if (driveR < (baseDrive - 30)) {
            if (driveR > 30) {
                driveR = baseDrive - 15;
            } else {
                driveR = baseDrive;
            }
        }
    }
}


//moves the robot forward one wheel rotation
bool rotato(uint8_t sixths) {

    tickRDiff = (rEncode - curRTicks);
    tickLDiff = (lEncode - curLTicks);

    if((tickLDiff <= (encTicksPerSixthWheelRev*sixths)) || (tickRDiff <= (encTicksPerSixthWheelRev*sixths))){
        driveStraight();
        return false;
    } else {
        robotDrive.botStop();
        //reset encoder values
        curLTicks = lEncode;
        curRTicks = rEncode;
        tickLDiff = 0;
        tickRDiff = 0;
        return true;
    }
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
            //reset encoder values
            curLTicks = lEncode;
            curRTicks = rEncode;
            tickLDiff = 0;
            tickRDiff = 0;
            return true;
            break;

        case IMU_TURN:
        gyroRead(gyro);
            if (gyro_z < IMU_TURN_90_LEFT) {
                robotDrive.botDrive(153, -153);
                return false;
            }
            robotDrive.botStop();
            LCD.setCursor(0,0);
            LCD.print(gyro_z);
            //reset the gyro
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
            //reset encoder values
            curLTicks = lEncode;
            curRTicks = rEncode;
            tickLDiff = 0;
            tickRDiff = 0;
            return true;
            break;

        case IMU_TURN:
            gyroRead(gyro);
            if (gyro_z > IMU_TURN_90_RIGHT) {
                robotDrive.botDrive(-153, 153);
                return false;
            }
            robotDrive.botStop();
            LCD.setCursor(0,0);
            LCD.print(gyro_z);
            //reset the gyro
            gyroGood = false;
            gyro_z = 0;
            gyro_zold = 0;
            return true;
            break;

        default:
            return false;
    }
}


//turns the robot 90 degrees to the left
bool turn5DegLeft(uint8_t deg5) {
    switch (turnState) {
        case ENCODER_TURN:
            tickRDiff = (rEncode - curRTicks);
            tickLDiff = (lEncode - curLTicks);

            if ((tickRDiff < (tickPer5Deg*deg5)) || (tickLDiff < (tickPer5Deg*deg5))) {
                robotDrive.botDrive(100, -100);
                return false;
            }
            robotDrive.botStop();
            //reset encoder values
            curLTicks = lEncode;
            curRTicks = rEncode;
            tickLDiff = 0;
            tickRDiff = 0;
            return true;
            break;

        case IMU_TURN:
        gyroRead(gyro);
            if (gyro_z < (IMU_TURN_5_LEFT * deg5)) {
                robotDrive.botDrive(100, -100);
                return false;
            }
            robotDrive.botStop();
            //reset the gyro
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
bool turn5DegRight(uint8_t deg5) {
    switch (turnState) {
        case ENCODER_TURN:
            tickRDiff = (rEncode - curRTicks);
            tickLDiff = (lEncode - curLTicks);

            if ((tickRDiff < (tickPer5Deg*deg5)) || (tickLDiff < (tickPer5Deg*deg5))) {
                robotDrive.botDrive(-100, 100);
                return false;
            }
            robotDrive.botStop();
            //reset encoder values
            curLTicks = lEncode;
            curRTicks = rEncode;
            tickLDiff = 0;
            tickRDiff = 0;
            return true;
            break;

        case IMU_TURN:
            gyroRead(gyro);
            if (gyro_z > (IMU_TURN_5_RIGHT * deg5)) {
                robotDrive.botDrive(-100, 100);
                return false;
            }
            robotDrive.botStop();
            //reset the gyro
            gyroGood = false;
            gyro_z = 0;
            gyro_zold = 0;
            return true;
            break;

        default:
            return false;
    }
}


//turns the robot 90 degrees to the left
bool turnSlightLeft(void) {
    tickRDiff = (rEncode - curRTicks);
    tickLDiff = (lEncode - curLTicks);

    if ((tickRDiff < tickPer90) || (tickLDiff < tickPer90)) {
        robotDrive.botDrive(50, -50);
        return false;
    }
    robotDrive.botStop();
    //reset encoder values
    curLTicks = lEncode;
    curRTicks = rEncode;
    tickLDiff = 0;
    tickRDiff = 0;
    return true;
}


//turns the robot 90 degrees to the right
bool turnSlightRight(void) {
    tickRDiff = (rEncode - curRTicks);
    tickLDiff = (lEncode - curLTicks);

    if ((tickRDiff < tickPer90) || (tickLDiff < tickPer90)) {
        robotDrive.botDrive(-50, 50);
        return false;
    }
    robotDrive.botStop();
    //reset encoder values
    curLTicks = lEncode;
    curRTicks = rEncode;
    tickLDiff = 0;
    tickRDiff = 0;
    return true;
}


//rotates the robot to scan for the candle flame
bool sweep(void) {
    uint8_t mod4 = sweepState % 4;
    switch(mod4) {
        case 0:
            if (turnRight90()) {
                sweepState++;
                delay(50);
            }
            break;

        case 1:
            if (turnLeft90()) {
                sweepState++;
                delay(50);
            }
            break;

        case 2:
            if (turnLeft90()) {
                sweepState++;
                delay(50);
            }
            break;

        case 3:
            if (turnRight90()) {
                blue.debugLEDON();
                delay(50);
                sweepState = 0;
                return true;
            }
            break;
    }
    return false;
}


/*************************************************************************************************************************/
//determines where a wall is, if there is a wall
uint8_t wallTest() {
    robotDrive.botStop();
    readAllUS();

    if ((USVals[2] < 25) && (USVals[2] > 0)) { //if a wall in front
        if ((USVals[0] > USVals[1]) && (USVals[1] > 0)) { //if a left wall is nearer than a right wall
                return TURN_RIGHT;
        } else { //if a right wall is nearer than a left wall or no wall is nearer
                return TURN_LEFT;
        }
    }

    if ((USVals[0] <= 15) && (USVals[0] > 0)) { //if a left wall is near
        return WALL_LEFT;
    }

    if ((USVals[1] <= 15) && (USVals[1] > 0)) { //if a right wall is near
        return WALL_RIGHT;
    }

    if ((USVals[0] <= 30) || (USVals[1] <= 30)) {

        if ((USVals[0] > 15) && (USVals[0] > 0)) { //if a left wall is near
            return NO_WALLS_RIGHT;
        }

        if ((USVals[1] > 15) && (USVals[1] > 0)) { //if a right wall is near
            return NO_WALLS_LEFT;
        }
        return FORWARD;
    }

    if ((USVals[0] > 30) && (USVals[1] > 30) && (USVals[2] > 30)) {
        return NO_WALLS;
    }

    return WALL_TEST;
}


//wall following function
void wallNav() {
    switch(wallState) {
        case WALL_TEST:
            blue.debugLEDOFF();
            robotDrive.botStop();
            wallState = wallTest();
            break;

        case TURN_RIGHT:
            if (turn5DegRight(53)) {
                wallState = WALL_TEST;
            }
            break;

        case TURN_LEFT:
            if (turn5DegLeft(53)) {
                wallState = WALL_TEST;
            }
            break;

        case WALL_RIGHT:
            driveL -= 10;
            driveR += 0;
            wallState = FORWARD;
            break;

        case WALL_LEFT:
            driveR -= 10;
            driveL += 0;
            wallState = FORWARD;
            break;

        case FORWARD:
            blue.debugLEDON();
            if (rotato(2)) {
                wallState = WALL_TEST;
            }
            break;

        case NO_WALLS:
            robotDrive.botStop();
            mazeState = MAZE_TEST;
            wallState = WALL_TEST;
            break;

        case NO_WALLS_RIGHT:
            if (driveL < driveR) {
                driveL = driveR + 5;
            } else {
                driveL = baseDrive;
                driveR = driveL - 5;
            }
            wallState = FORWARD;
            break;

        case NO_WALLS_LEFT:
            if (driveR < driveL) {
                driveR = driveL + 5;
            } else {
                driveR = baseDrive;
                driveL = driveR - 5;
            }
            wallState = FORWARD;
            break;

        default:
            Serial.println("Conrgrats");
            wallState = WALL_TEST;
            break;
    }
}


/*************************************************************************************************************************/
//sweeps back and forth to find the candle
void candleFind(void) {

    switch(candleState) {
        case CANDLE_FIND:
            fireExtinguisher.servoTilt(60);
            LCD.setCursor(0,0);
            LCD.print("FIND");
            candleState = candleTest();
            break;

        case CANDLE_NOT_FOUND:
            LCD.setCursor(0,0);
            LCD.print("No find");
            robotDrive.botDrive(-100, 100);
            candleState = candleTest();
            break;

        case CANDLE_FOUND:
            LCD.setCursor(0,0);
            LCD.print("Found");
            if (rotato(6)) {
                readAllUS();
                LCD.setCursor(0,1);
                LCD.print(USVals[2]);
                if ((USVals[2] > 0) && (USVals[2] < 17)) {
                    candleState = EXTINGUISH_CANDLE;
                }
            }
            break;

        case EXTINGUISH_CANDLE:
            robotDrive.botStop();
            for (int i = servoMinimum; i < servoMaximum; i++) {
                fireExtinguisher.servoTilt(i);
                delay(30);
                flameVals[i - servoMinimum] = fireExtinguisher.readFlameSense();
            }
            tempServoMin = 1024;
            for (int i = 0; i < 90; i++) {
                if (flameVals[i] < tempServoMin) {
                    tempServoMin = flameVals[i];
                    servoIndex = i;
                }
            }
            LCD.print(servoIndex);
            fireExtinguisher.servoTilt(servoIndex + servoMinimum);
            delay(100);
            LCD.setCursor(0,0);
            LCD.print("Extinguishing");
            fireExtinguisher.extinguishFire();
            if (fireExtinguisher.readFlameSense() > 980) {
                candleZ();
                candleState = FIRE_EXTINGUISHED;
                if (fireExtinguisher.findFlame()) {
                    candleState = EXTINGUISH_CANDLE;
                }
            } else {
                robotDrive.botDrive(-25, 25);
            }
            break;

        case FIRE_EXTINGUISHED:
            robotDrive.botStop();
            if (!drawn) {
                LCD.clear();
                LCD.setCursor(0,0);
                LCD.print("Flame out");
                LCD.setCursor(0,1);
                LCD.print(zPos);
                drawn = true;
            }
            botState = CALCULATE_VALUES;
            break;

        default:
            candleState = CANDLE_FIND;
            break;
    }
}


//sweeps back and forth to search for the candle
uint8_t candleTest(void) {

    if (fireExtinguisher.readFlameSense() < 600) {
        robotDrive.botStop();
        return CANDLE_FOUND;
    }

    if (!fireExtinguisher.readFlameSenseDig()) {
        robotDrive.botStop();
        return CANDLE_FOUND;
    }

    return CANDLE_NOT_FOUND;

}


/*************************************************************************************************************************/
//initializes the gyro
bool gyroSetup(L3G gyro) {
    delay(50);
    gyro.enableDefault(); // gyro init. default 250/deg/s
    delay(1000);// allow time for gyro to settle
    for(int i =0;i<100;i++){  // takes 100 samples of the gyro
        gyro.read();
        gerrz+=gyro.g.z;
        delay(25);
    }
    gerrz = gerrz/100;
    return true;
}


//reads the value of the gyro
void gyroRead(L3G gyro) {
    Wire.begin();
    gyro.init();
    if (!gyroGood) {
        gyroGood = gyroSetup(gyro);
    }
    if(timer % 5) { // reads imu every 5ms
        gyro.read(); // read gyro
        gyro_z=(float)(gyro.g.z-gerrz)*G_gain;
        gyro_z = gyro_z*G_Dt;
        if (abs(gyro_z - gyro_zold) > minErr) {
            gyro_z +=gyro_zold;
        }
        gyro_zold=gyro_z ;
    }
}


/*************************************************************************************************************************/
//navigates through the maze
void mazeSearch(void) {
    switch (mazeState) {
        case MAZE_TEST:
            robotDrive.botStop();
            LCD.setCursor(0,0);
            LCD.print("TESTING");
            mazeState = mazeWallTest();
            break;

        case WALL_AVOID:
            LCD.setCursor(0,0);
            LCD.print("Get out");
            wallNav();
            break;

        case SCAN_FOR_FIRE:
            LCD.setCursor(0,0);
            LCD.print("Scanning");
            if (!fireExtinguisher.readFlameSenseDig()) {
                robotDrive.botStop();
                mazeState = FIRE_DETECTED;
            } else {
                if (sweep()) {
                    mazeState = mazeWallTest();
                }
            }
            break;

        case STEP_FORWARD:
            LCD.setCursor(0,0);
            LCD.print("Forward");
            mazeState = mazeWallTest();
            if (rotato(12)) {
                mazeState = MAZE_TEST;
            }

        case FIRE_DETECTED:
            robotDrive.botStop();
            LCD.setCursor(0,0);
            LCD.print("Fire detected");
            botState = FIND_CANDLE;
            break;
    }
}


//returns the state to go to for maze searching
uint8_t mazeWallTest(void) {
    readAllUS();
    if (((USVals[2] > 0) && (USVals[2] < 25)) ||
            ((USVals[0] > 0) && (USVals[0] < 25)) ||
            ((USVals[1] > 0) && (USVals[1] < 25))) { //if there is a wall near
                return WALL_AVOID;
    }
    if (mazeState == MAZE_TEST) {
        return SCAN_FOR_FIRE;
    } else if (mazeState == STEP_FORWARD) {
        return STEP_FORWARD;
    }
}
