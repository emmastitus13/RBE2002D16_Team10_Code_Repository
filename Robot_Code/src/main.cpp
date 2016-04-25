/* RBE2002D16 Final Project Code
 *
 * Will not wall follow since the flame sensor cannot rotate independently of the robot
 * Will find the candle by exploratory methods :D
 *
 *
 *
 * Created on Apr 12. 2016 by Ben Titus
 * Last edit made Apr 24, 2016 by Ben Titus
 */

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
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

bool oneRotato(void);
bool turnLeft90(void);
bool turnRight90(void);
bool turnLeft90Slow(void);
bool turnRight90Slow(void);
bool turnSlightLeft(void);
bool turnSlightRight(void);
void driveStraight(void);

unsigned long readUS(NewPing us);
void readAllUS(void);

uint8_t wallTest(void);
void wallNav(void);

void candleFind(void);
uint8_t candleTest(void);
uint8_t candleTestHigh(void);
void candleSweep(void);
void candleSlowSweep(void);
void candleSlightSweep(void);

bool gyroSetup(L3G gyro);
void gyroRead(L3G gyro);

void logMove(void);
void candleZ(void);


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


//states
uint8_t wallState = WALL_TEST;
volatile uint8_t botState = STOP;
uint8_t candleState = CANDLE_FIND;
uint8_t turnState = IMU_TURN;
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

//general variables
bool upDown;
bool gyroGood = false;
uint8_t ii;

//driving variables
volatile bool frBumpPush = false;
uint8_t baseDrive = 255;
uint8_t driveL = baseDrive;
uint8_t driveR = baseDrive;

//movement variables
uint8_t globi = 0;
int buffTicks= 0;
float tempAngle = 0.0;
Movement movBuf = {globi, buffTicks, tempAngle};

//gyro variables
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
int flameVals[ARRAY_LENGTH];


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
    candleFind();
    // readAllUS();
    // LCD.setCursor(0,0);
    // LCD.print("Finding flame");
    // fireExtinguisher.findFlame();
    // LCD.clear();
    // LCD.setCursor(0,0);
    // LCD.print("Extinghuishing");
    // LCD.setCursor(0,1);
    // LCD.print("Fire");
    // fireExtinguisher.extinguishFire();
    // LCD.clear();
    // LCD.setCursor(0,0);
    // LCD.print("Fire is out");
    // LCD.setCursor(0,1);
    // LCD.print("Pos:");
    // LCD.print(fireExtinguisher.servoPosToAngle());
    // LCD.print(" dist:");
    // LCD.print(USVals[2]);
    // delay(1000000);
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
    float angle = fireExtinguisher.servoPosToAngle();
    float dist = USVals[2] + fireExtinguisher.distCorrect();
    zPos = (dist * tan(angle)) + 0.5;
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
            driveR--;
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
    if (test) {
        Serial.print(lEncode);
        Serial.print(" ");
        Serial.println(rEncode);
    }

    tickRDiff = (rEncode - curRTicks);
    tickLDiff = (lEncode - curLTicks);

    if((tickLDiff <= (encTicksPerWheelRev / 2)) || (tickRDiff <= (encTicksPerWheelRev / 2))){
        wallState = wallTest();
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
            gyroRead(gyro);
            tickRDiff = (rEncode - curRTicks);
            tickLDiff = (lEncode - curLTicks);

            if ((tickRDiff < tickPer90) || (tickLDiff < tickPer90)) {
                robotDrive.botTurnLeft();
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
                robotDrive.botTurnLeft();
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
bool turnRight90(void) {
    switch (turnState) {
        case ENCODER_TURN:
            gyroRead(gyro);
            tickRDiff = (rEncode - curRTicks);
            tickLDiff = (lEncode - curLTicks);

            if ((tickRDiff < tickPer90) || (tickLDiff < tickPer90)) {
                robotDrive.botTurnRight();
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
                robotDrive.botTurnRight();
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
bool turnLeft90Slow(void) {
    switch (turnState) {
        case ENCODER_TURN:
            gyroRead(gyro);
            tickRDiff = (rEncode - curRTicks);
            tickLDiff = (lEncode - curLTicks);

            if ((tickRDiff < tickPer90) || (tickLDiff < tickPer90)) {
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
            if (gyro_z < IMU_TURN_90_LEFT) {
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
bool turnRight90Slow(void) {
    switch (turnState) {
        case ENCODER_TURN:
            gyroRead(gyro);
            tickRDiff = (rEncode - curRTicks);
            tickLDiff = (lEncode - curLTicks);

            if ((tickRDiff < tickPer90) || (tickLDiff < tickPer90)) {
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
            if (gyro_z > IMU_TURN_90_RIGHT) {
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


//determines where a wall is, if there is a wall
uint8_t wallTest() {
    readAllUS();
    //if one side is too slow
    if (driveL < (baseDrive - 15)) {
        driveL = baseDrive - 5;
    }
    if (driveR < (baseDrive - 15)) {
        driveR = baseDrive - 5;
    }

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

    return WALL_TEST;
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
                logMove();
                wallState = WALL_TEST;
            } else {}
            break;

        case TURN_LEFT:
            if (turnLeft90()) {
                logMove();
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

        default:
            Serial.println("Conrgrats");
            wallState = WALL_TEST;
            break;
    }
}


//sweeps back and forth to find the candle
void candleFind(void) {
    LCD.setCursor(0,0);
    LCD.print(candleState);
    fireExtinguisher.servoTilt(60);
    blue.debugLEDOFF();

    switch(candleState) {
        case CANDLE_FIND:
            LCD.setCursor(0,0);
            LCD.print("FIND");
            candleState = candleTest();
            break;

        case CANDLE_NOT_FOUND:
            LCD.setCursor(0,0);
            LCD.print("No find");
            turnRight90();
            candleState = candleTest();
            break;

        case CANDLE_FOUND:
            LCD.setCursor(0,0);
            LCD.print("Found");
            readAllUS();
            if ((USVals[2] > 0) && (USVals[2] < 25)) {
                candleState = LINE_IT_UP;
            }
            robotDrive.botStop();
            if (oneRotato()) {
                candleState = candleTest();
            }
            break;

        case LINE_IT_UP:
            LCD.setCursor(0,0);
            LCD.print("Very Strong");
            if (oneRotato()) {
                robotDrive.botStop();
                if (oneRotato()) {
                    candleState = candleTest();
                }
                readAllUS();
                LCD.setCursor(0,1);
                LCD.print(USVals[2]);
                if ((USVals[2] > 0) && (USVals[2] < 25)) {
                    candleState = EXTINGUISH_CANDLE;
                    blue.debugLEDON();
                }
                if (fireExtinguisher.readFlameSense() < 150) {
                    candleState = LINE_IT_UP_AGAIN;
                    orange.debugLEDON();
                }
            }
            break;

        case LINE_IT_UP_AGAIN:
            //candleSlightSweep();
            flameVals[ii] = fireExtinguisher.readFlameSense();
            if (ii > 3) {
                if (flameVals[ii] < flameVals[ii-3]) {
                    robotDrive.botStop();
                    candleState = EXTINGUISH_CANDLE;
                }
            }
            ii++;
            break;

        case EXTINGUISH_CANDLE:
            robotDrive.botStop();
            LCD.setCursor(0,0);
            LCD.print("Extinguishing");
            fireExtinguisher.findFlame();
            fireExtinguisher.extinguishFire();
            if (fireExtinguisher.readFlameSense() > 980) {
                candleState = FIRE_EXTINGUISHED;
            }
            oneRotato();
            break;

        case FIRE_EXTINGUISHED:
            robotDrive.botStop();
            LCD.setCursor(0,0);
            LCD.print("Flame out");
            break;

        default:
            wallState = CANDLE_FIND;
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
        blue.debugLEDON();
        return CANDLE_FOUND;
    }

    return CANDLE_NOT_FOUND;

}


//sweeps back and forth to search for the candle
uint8_t candleTestHigh(void) {
    robotDrive.botStop();
    readAllUS();
    bool dig = fireExtinguisher.readFlameSenseDig();
    int an = fireExtinguisher.readFlameSense();

    candleSlightSweep();

    if ((USVals[2] > 0) && (USVals[2] < 25)) {
        return LINE_IT_UP;
    }

    if (an < 200) {
        return LINE_IT_UP;
    }

    if (an < 600) {
        return CANDLE_FOUND;
    }

    if (dig) {
        return CANDLE_FOUND;
    }

    return CANDLE_NOT_FOUND;

}


//rotates the robot to scan for the candle flame
void candleSweep(void) {
    switch(sweepState % 4) {
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
                sweepState++;
                delay(50);
            }
            break;
    }
}


//rotates the robot to scan for the candle flame
void candleSlowSweep(void) {
    switch(slowSweepState % 4) {
        case 0:
            if (turnRight90Slow()) {
                slowSweepState++;
                delay(50);
            }
            break;

        case 1:
            if (turnLeft90Slow()) {
                slowSweepState++;
                delay(50);
            }
            break;

        case 2:
            if (turnLeft90Slow()) {
                slowSweepState++;
                delay(50);
            }
            break;

        case 3:
            if (turnRight90Slow()) {
                slowSweepState++;
                delay(50);
            }
            break;
    }
}


//sweeps a small amount to the right and left using encoders
void candleSlightSweep(void) {
    switch(slightSweepState % 4) {
        case 0:
            if (turnRight90Slow()) {
                slightSweepState++;
                delay(50);
            }
            break;

        case 1:
            if (turnLeft90Slow()) {
                slightSweepState++;
                delay(50);
            }
            break;

        case 2:
            if (turnLeft90Slow()) {
                slightSweepState++;
                delay(50);
            }
            break;

        case 3:
            if (turnRight90Slow()) {
                slightSweepState++;
                delay(50);
            }
            break;
    }
}


//initializes the gyro
bool gyroSetup(L3G gyro) {
    delay(50);
    gyro.enableDefault(); // gyro init. default 250/deg/s
    delay(1000);// allow time for gyro to settle
    orange.debugLEDON();
    for(int i =0;i<100;i++){  // takes 100 samples of the gyro
        gyro.read();
        gerrz+=gyro.g.z;
        delay(25);
    }
    gerrz = gerrz/100;

    if (test) {
        Serial.println(gerrz);
    }
    orange.debugLEDOFF();
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

    if (test) {// prints the gyro value once per second
        gyroTimer1=millis();
        LCD.setCursor(0,1);
        LCD.print("Z: ");
        LCD.println(gyro_z);
    }
}
