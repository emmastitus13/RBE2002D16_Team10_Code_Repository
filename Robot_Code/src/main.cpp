#include <Arduino.h>
#include "driveFunctions.cpp"
#include "definitions.cpp"
#include "fireExtinguisher.cpp"
#include "hBridgeMotorDriver.h"
#include "NewPing.h"

bool test = true;

void lEncoderISR(void);
void rEncoderISR(void);


NewPing leftUS(LEFT_US_TP, LEFT_US_EP, MAX_DISTANCE);
NewPing rightUS(RIGHT_US_TP, RIGHT_US_EP, MAX_DISTANCE);
NewPing forwardUS(FORWARD_US_TP, FORWARD_US_EP, MAX_DISTANCE);

volatile unsigned char botState = STOP;
volatile unsigned long lEncode = 0;
volatile unsigned long rEncode = 0;
unsigned long currentL = 0, currentR = 0;

Type2_Motor leftMotor(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2);
Type2_Motor rightMotor(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2);

void setup() {
    Serial.begin(115200);

    pinMode(L_ENCODER_PIN, INPUT_PULLUP);
    pinMode(R_ENCODER_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN), lEncoderISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN), rEncoderISR, FALLING);

}

void loop() {
    if (test) {
        delay(50);
        int uS = leftUS.ping();
        Serial.print("Ping: ");
        Serial.print(uS / US_ROUNDTRIP_CM);
        Serial.println("cm");
        /*
        leftMotor.driveBackward(255);
        rightMotor.driveForward(255);
        Serial.print("Left Encoder: ");
        Serial.print((lEncode - currentL) / 500);
        Serial.print(" Right Encoder: ");
        Serial.println((rEncode - currentR) / 500);
        delay(500);
        currentL = lEncode;
        currentR = rEncode;*/
    } else {
        switch (botState) {
            case STOP:
                break;

        }
    }
}


void lEncoderISR(void) {
    lEncode++;
}
void rEncoderISR(void) {
    rEncode++;
}
