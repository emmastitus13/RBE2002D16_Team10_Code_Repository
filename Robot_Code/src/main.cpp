/* RBE2002D16 Final Project Code
 *
 * Will not wall follow since the flame sensor cannot rotate independently of the robot
 * Will find the candle by exploratory methods
 *
 *
 *
 * Created on Apr 12. 2016 by Ben Titus
 * Last edit made Apr 17, 2016 by Ben Titus
 */

#include <Arduino.h>
#include <Servo.h>
#include "TimerOne.h"
#include "definitions.cpp"
#include "DriveTrain.h"
#include "FireExtinguisher.h"
#include "hBridgeMotorDriver.h"
#include "NewPing.h"

void findAndExtinguishCandle(void);
void lEncoderISR(void);
void rEncoderISR(void);
void timer1ISR(void);
void frontBumpISR(void);


NewPing leftUS(LEFT_US_TP, LEFT_US_EP, MAX_DISTANCE);
NewPing rightUS(RIGHT_US_TP, RIGHT_US_EP, MAX_DISTANCE);
NewPing forwardUS(FORWARD_US_TP, FORWARD_US_EP, MAX_DISTANCE);
DriveTrain robotDrive(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, MAX_MOTOR_SPEED);
FireExtinguisher fireExtinguisher(FAN_PIN, FLAME_SENSE_PINA, FLAME_SENSE_PIND, TILT_SERVO_PIN, FLAME_SENSOR_CONSTANT);

NewPing USSensors[3] ={leftUS, forwardUS, rightUS};

volatile unsigned char botState = STOP;
volatile unsigned long lEncode = 0;
volatile unsigned long rEncode = 0;
volatile unsigned int timer1cnt = 0;
unsigned int timer = 0;
unsigned long currentL = 0, currentR = 0;
int servoMaximum, servoMinimum, servoPosition;
bool upDown;
bool fBumpPush = false;

/*************************************************************************************************************************/
void setup() {
    Serial.begin(9600);

    Timer1.initialize(10000); //10ms timer to time some things. open to changes
    Timer1.attachInterrupt(timer1ISR);

    pinMode(L_ENCODER_PIN, INPUT_PULLUP);
    pinMode(R_ENCODER_PIN, INPUT_PULLUP);
    pinMode(FRONT_BUMPER, INPUT);

    attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN), lEncoderISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN), rEncoderISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(FRONT_BUMPER), frontBumpISR, RISING);

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
    if (fBumpPush) {
        //do nothing
    } else {
        robotDrive.botDrive(100, 100);
        if ((timer1cnt % 500) / 2) {
            Serial.print("Leftt: ");
            Serial.print(lEncode);
            Serial.print(" Right: ");
            Serial.println(rEncode);
        }
    }
}

/*************************************************************************************************************************/
//ISR for the 10ms timer
void timer1ISR(void) {
    timer1cnt++;
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
    fBumpPush = true;
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



//records a movement
//movement consists of an angle, a number of ticks
//movements recorded in array of movements

//reads the values of a US sensor

//drives the robot straight according to the encoders

//returns the distance of a movement

//turns the robot 90 degrees based on encoders

//If you think of anything else, add it here
