/* RBE2002D16 Final Project Code
 *
 *
 *
 *
 *
 *
 * Created on Apr 12. 2016 by Ben Titus
 * Last edit made Apr 16, 2016 by Ben Titus
 */

#include <Arduino.h>
#include <Servo.h>
#include "definitions.cpp"
#include "DriveTrain.h"
#include "FireExtinguisher.h"
#include "hBridgeMotorDriver.h"
#include "NewPing.h"

void findAndExtinguishCandle(void);
void lEncoderISR(void);
void rEncoderISR(void);


NewPing leftUS(LEFT_US_TP, LEFT_US_EP, MAX_DISTANCE);
NewPing rightUS(RIGHT_US_TP, RIGHT_US_EP, MAX_DISTANCE);
NewPing forwardUS(FORWARD_US_TP, FORWARD_US_EP, MAX_DISTANCE);
DriveTrain robotDrive(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, MAX_MOTOR_SPEED);
FireExtinguisher fireExtinguisher(FAN_PIN, FLAME_SENSE_PINA, FLAME_SENSE_PIND, TILT_SERVO_PIN, FLAME_SENSOR_CONSTANT);

NewPing USSensors[3] ={leftUS, forwardUS, rightUS};

volatile unsigned char botState = STOP;
volatile unsigned long lEncode = 0;
volatile unsigned long rEncode = 0;
unsigned long currentL = 0, currentR = 0;
int servoMaximum, servoMinimum, servoPosition;
bool upDown;

void setup() {
    Serial.begin(115200);

    pinMode(L_ENCODER_PIN, INPUT_PULLUP);
    pinMode(R_ENCODER_PIN, INPUT_PULLUP);
    pinMode(7, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN), lEncoderISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN), rEncoderISR, FALLING);
    analogWrite(7, 50);

    fireExtinguisher.setServo(30, 120);
    robotDrive.attachMotors();

    servoMaximum = fireExtinguisher.servoMax;
    servoMinimum = fireExtinguisher.servoMin;
    servoPosition = fireExtinguisher.servoPos;
    upDown = true;

    robotDrive.botStop();
    fireExtinguisher.fanOff();
}

void loop() {
    if ((servoPosition < servoMaximum) & upDown) {
        servoPosition++;
        fireExtinguisher.servoTilt(servoPosition);
    }

    if (servoPosition >= servoMaximum) {
        upDown = false;
        robotDrive.botStop();
        fireExtinguisher.fanOn();
    }

    if ((servoPosition > servoMinimum) & !upDown) {
        servoPosition--;
        fireExtinguisher.servoTilt(servoPosition);
    }

    if (servoPosition <= servoMinimum) {
        upDown = true;
        fireExtinguisher.fanOff();
        robotDrive.botDrive(100, 100);
    }
    delay(5);
    /*
    fireExtinguisher.fanOn();
    delay(500);
    fireExtinguisher.fanOff();
    */
    /*
    delay(50);
    int uS = leftUS.ping();
    Serial.print("Ping: ");
    Serial.print(uS / US_ROUNDTRIP_CM);
    Serial.println("cm");*/

}

void lEncoderISR(void) {
    lEncode++;
}
void rEncoderISR(void) {
    rEncode++;
}

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
