#include "Robot.h"



Robot::Robot(LiquidCrystal& lcd, FireExtinguisher& fx, DriveTrain& rb):
	LCD(lcd)
	,fireExtinguisher(fx)
	,robotDrive(rb){

}


void  Robot::updatePosition(int newX, int newY, int newZ) {
    char pos[10];
    sprintf(pos, "%2d, %2d, %2d", newX, newY, newZ);
    LCD.setCursor(0,0);
    LCD.print("Flame Position:");
    LCD.setCursor(0,1);
    LCD.print(pos);
}


void Robot::logMove(Movement mov) {
    movements[globi] = mov;
    globi++;
}

uint8_t Robot::mazeWallTest(void) {
    fireExtinguisher.servoTilt(60);
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

//sweeps back and forth to search for the candle
uint8_t Robot::candleTest(void) {

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

//reads all three US sensors
void Robot::readAllUS(void) {
        USVals[0] = leftUS.ping_cm();
        delay(50);

        USVals[1] = rightUS.ping_cm();
        delay(50);

        USVals[2] = frontUS.ping_cm();
        delay(50);
}

//prints the values of the ultrasonic sensors
void Robot::printAllUS(void) {
    readAllUS();
    LCD.clear();
    LCD.setCursor(0,0);
    LCD.print("L: ");
    LCD.print(USVals[0]);
    LCD.print("F: ");
    LCD.print(USVals[2]);
    LCD.print("R: ");
    LCD.print(USVals[1]);
    delay(200);
}

//calculates the height of the candle
void Robot::candleZ(void) {
    readAllUS();
    float angle = fireExtinguisher.servoPosToAngle() * PI / 180;
    zPos = ((USVals[2] + 5) * 2 / 3 * tan(angle)) + 19;
}