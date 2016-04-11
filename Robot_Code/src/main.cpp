#include <Arduino.h>
#include "driveFunctions.cpp"
#include "H-BridgeMotorDriver.h"
#include ""

volatile unsigned char botState = STOP;

Type1_Motor leftMotor, rightMotor, fans;


void setup() {

}

void loop() {
    switch (botState) {
        case STOP:

    }
}
