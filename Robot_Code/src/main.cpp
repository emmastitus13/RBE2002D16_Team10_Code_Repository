/* RBE2002D16 Final Project Code
 *
 * Will not wall follow since the flame sensor cannot rotate independently of the robot
 * Will find the candle by exploratory methods
 *
 *
 *
 * Created on Apr 12. 2016 by Ben Titus
 * Last edit made Apr 20, 2016 by Ben Titus
 */

#include <Arduino.h>
#include <Servo.h>
#include "LiquidCrystal.h"
#include "TimerOne.h"
#include "definitions.cpp"
#include "DriveTrain.h"
#include "FireExtinguisher.h"
#include "hBridgeMotorDriver.h"
#include "NewPing.h"

bool test = true;

void logMove();
unsigned long readUS(NewPing us);
void readAllUS(void);
void findAndExtinguishCandle(void);
void lEncoderISR(void);
void rEncoderISR(void);
void timer1ISR(void);
void frontBumpISR(void);


NewPing leftUS(LEFT_US_TP, LEFT_US_EP, MAX_DISTANCE);
NewPing rightUS(RIGHT_US_TP, RIGHT_US_EP, MAX_DISTANCE);
NewPing frontUS(FORWARD_US_TP, FORWARD_US_EP, MAX_DISTANCE);
DriveTrain robotDrive(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, MAX_MOTOR_SPEED);
FireExtinguisher fireExtinguisher(FAN_PIN, FLAME_SENSE_PINA, FLAME_SENSE_PIND, TILT_SERVO_PIN, FLAME_SENSOR_CONSTANT);
LiquidCrystal LCD(RS_PIN, EN_PIN, DB1_PIN, DB2_PIN, DB3_PIN, DB4_PIN);

volatile unsigned char botState = STOP;
volatile unsigned long lEncode = 0;
volatile unsigned long rEncode = 0;
volatile unsigned int timer1cnt = 0;
unsigned int timer = 0;
unsigned long currentL = 0, currentR = 0;
unsigned long lUSVal, rUSVal, frUSVal;
int servoMaximum, servoMinimum, servoPosition;
bool upDown;
bool frBumpPush = false;
int pastlEnc = 0, pastrEnc = 0;
uint8_t baseDrive = 255;
uint8_t driveL = baseDrive;
uint8_t driveR = baseDrive;

NewPing USSensors[3] ={leftUS, frontUS, rightUS};
unsigned long USVals[3] = {lUSVal, rUSVal, frUSVal};

/*************************************************************************************************************************/
void setup() {
    Serial.begin(115200);

    Timer1.initialize(1000); //10ms timer to time some things. open to changes
    Timer1.attachInterrupt(timer1ISR);

    pinMode(L_ENCODER_PIN, INPUT_PULLUP);
    pinMode(R_ENCODER_PIN, INPUT_PULLUP);
    pinMode(FRONT_BUMPER, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN), lEncoderISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN), rEncoderISR, FALLING);
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
    if ((timer1cnt % 50) / 2) {
        readAllUS();

        if ((USVals[0] < 10) && (USVals[0] > 0)) {
            driveL++;
        }

        if ((USVals[1] < 10) && (USVals[1] > 0)) {
            driveR++;
        }

        if ((USVals[2] < 10) && (USVals[2] > 0)) {
            if (USVals[0] > USVals[1]) {
                if ((USVals[1] < 10) && (USVals[1] > 0)) {

                } else {
                    robotDrive.botTurnLeft();
                }
            }
        }
    }


    if (timer >= 500) {
        timer = 0;
        Serial.print("Left US: ");
        Serial.print(USVals[0]);
        Serial.print(" Right US: ");
        Serial.print(USVals[1]);
        Serial.print(" Front US: ");
        Serial.println(USVals[2]);
    }
    timer = timer1cnt;
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

}


//reads the values of a US sensor
unsigned long readUS(NewPing us) {
    return us.ping_cm();
}


//reads all three US sensors
void readAllUS(void) {
        USVals[0] = leftUS.ping_cm();
        delay(15);

        USVals[1] = rightUS.ping_cm();
        delay(15);

        USVals[2] = frontUS.ping_cm();
        delay(15);
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

//returns the distance of a movement

//turns the robot 90 degrees based on encoders

//If you think of anything else, add it here
