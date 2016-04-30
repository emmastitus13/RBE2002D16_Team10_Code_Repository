#include "Robot.h"



Robot::Robot(LiquidCrystal& lcd, FireExtinguisher& fx, DriveTrain& rb, NewPing& lus, NewPing& rus, NewPing& fus, DebugLED& _orange, DebugLED& _blue, Gyro& _gyro):
	LCD(lcd)
	,fireExtinguisher(fx)
	,robotDrive(rb)
	,leftUS(lus),rightUS(rus),frontUS(fus)
	,orange(_orange),blue(_blue)
	,gyro(_gyro){

}


void  Robot::updatePosition(int newX, int newY, int newZ) {
    char pos[10];
    sprintf(pos, "%2d, %2d, %2d", newX, newY, newZ);
    LCD.setCursor(0,0);
    LCD.print("Flame Position:");
    LCD.setCursor(0,1);
    LCD.print(pos);
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


//navigates through the maze
bool Robot::mazeSearch(void) {
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
            if (wallNav()) {
                mazeState = MAZE_TEST;
            }
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
            return true;
            break;
    }
    return false;
}


//moves the robot forward one wheel rotation
bool Robot::rotato(uint8_t sixths) {

    tickRDiff = (rEncode - curRTicks);
    tickLDiff = (lEncode - curLTicks);

    if((tickLDiff <= (encTicksPerSixthWheelRev*sixths)) || (tickRDiff <= (encTicksPerSixthWheelRev*sixths))){
        driveStraight();
        return false;
    } else {
        robotDrive.botStop();
        movBuf.encTicks += min(lEncode, rEncode);
        //reset encoder values
        curLTicks = lEncode;
        curRTicks = rEncode;
        tickLDiff = 0;
        tickRDiff = 0;
        return true;
    }
}

//drives the robot straight according to the encoders
void Robot::driveStraight() {
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

//rotates the robot to scan for the candle flame
bool Robot::sweep(void) {
    uint8_t mod4 = sweepState % 4;
    switch(mod4) {
        case 0:
            if (turn5DegRight(12)) {
                sweepState++;
                delay(50);
            }
            break;

        case 1:
            if (turn5DegLeft(12)) {
                sweepState++;
                delay(50);
            }
            break;

        case 2:
            if (turn5DegLeft(12)) {
                sweepState++;
                delay(50);
            }
            break;

        case 3:
            if (turn5DegRight(12)) {
                blue.debugLEDON();
                delay(50);
                sweepState = 0;
                return true;
            }
            break;
    }
    return false;
}

//turns the robot 90 degrees to the left
bool Robot::turn5DegLeft(uint8_t deg5) {
    switch (turnState) {
        case ENCODER_TURN:
            tickRDiff = (rEncode - curRTicks);
            tickLDiff = (lEncode - curLTicks);

            if ((tickRDiff < (tickPer5Deg*deg5)) || (tickLDiff < (tickPer5Deg*deg5))) {
                robotDrive.botDrive(100, -100);
                return false;
            }
            robotDrive.botStop();
            movBuf.angle += tempAngle;
            logMove(movBuf);
            //reset encoder values
            curLTicks = lEncode;
            curRTicks = rEncode;
            tickLDiff = 0;
            tickRDiff = 0;
            return true;
            break;

        case IMU_TURN:
        gyro.gyroRead();
            if (gyro_z < (IMU_TURN_5_LEFT * deg5)) {
                robotDrive.botDrive(100, -100);
                tempAngle = tempAngle * gyro_z / (IMU_TURN_5_LEFT * deg5);
                return false;
            }
            robotDrive.botStop();
            movBuf.angle += tempAngle;
            logMove(movBuf);
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
bool Robot::turn5DegRight(uint8_t deg5) {
    switch (turnState) {
        case ENCODER_TURN:
            tickRDiff = (rEncode - curRTicks);
            tickLDiff = (lEncode - curLTicks);

            if ((tickRDiff < (tickPer5Deg*deg5)) || (tickLDiff < (tickPer5Deg*deg5))) {
                robotDrive.botDrive(-100, 100);
                tempAngle = tempAngle * gyro_z / (IMU_TURN_5_LEFT * deg5);
                return false;
            }
            robotDrive.botStop();
            movBuf.angle += tempAngle;
            logMove(movBuf);
            //reset encoder values
            curLTicks = lEncode;
            curRTicks = rEncode;
            tickLDiff = 0;
            tickRDiff = 0;
            return true;
            break;

        case IMU_TURN:
            gyro.gyroRead();
            if (gyro_z > (IMU_TURN_5_RIGHT * deg5)) {
                robotDrive.botDrive(-100, 100);
                return false;
            }
            robotDrive.botStop();
            movBuf.angle += tempAngle;
            logMove(movBuf);
            //reset encoder values
            curLTicks = lEncode;
            curRTicks = rEncode;
            tickLDiff = 0;
            tickRDiff = 0;
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


void Robot::logMove(Movement mov) {
    movements[globi] = mov;
    globi++;
}


//wall following function
bool Robot::wallNav() {
    switch(wallState) {
        case WALL_TEST:
            blue.debugLEDOFF();
            orange.debugLEDOFF();
            robotDrive.botStop();
            wallState = wallTest();
            break;

        case TURN_LEFT:
            tempAngle = 90;
            if (turn5DegRight(18)) {
                wallState = WALL_TEST;
            }
            break;

        case TURN_RIGHT:
            tempAngle = 90;
            if (turn5DegLeft(18)) {
                wallState = WALL_TEST;
            }
            break;

        case WALL_RIGHT:
            orange.debugLEDON();
            if (driveR <= driveL) {
                driveL = driveR - 9;
            } else {
                driveL = driveR;
            }
            wallState = FORWARD;
            break;

        case WALL_LEFT:
            blue.debugLEDON();
            if (driveL <= driveR) {
                driveR = driveL - 9;
            } else {
                driveR = driveL;
            }
            wallState = FORWARD;
            break;

        case FORWARD:
            if (rotato(9)) {
                wallState = WALL_TEST;
                if (wallCount >= 5) {
                    wallState = WALL_SCAN;
                }
                wallCount++;
            }
            break;

        case WALL_SCAN:
            wallCount = 0;
            if (wallSweep(USVals[0] < USVals[1])) {
                wallState = WALL_TEST;
            }
            if (!fireExtinguisher.readFlameSenseDig()) {
                return true;
            }
            break;

        case NO_WALLS:
            if (aroundWall()) {
                wallState = WALL_TEST;
            }
            break;

        case NO_WALLS_RIGHT:
            if (driveL < driveR) {
                driveL = driveR - 9;
            } else {
                driveL = baseDrive;
                driveR = driveL + 9;
            }
            wallState = FORWARD;
            break;

        case NO_WALLS_LEFT:
            if (driveR < driveL) {
                driveR = driveL - 9;
            } else {
                driveR = baseDrive;
                driveL = driveR + 9;
            }
            wallState = FORWARD;
            break;

        default:
            Serial.println("Conrgrats");
            wallState = WALL_TEST;
            break;
    }
    return false;
}


/*************************************************************************************************************************/
//determines where a wall is, if there is a wall
uint8_t Robot::wallTest() {
    robotDrive.botStop();
    readAllUS();

    if ((USVals[2] < 13) && (USVals[2] > 0)) { //if a wall in front
        if ((USVals[0] > USVals[1]) && (USVals[1] > 0)) { //if a left wall is nearer than a right wall
                return TURN_RIGHT;
        } else { //if a right wall is nearer than a left wall or no wall is nearer
                return TURN_LEFT;
        }
    }

    if ((USVals[0] <= 10) && (USVals[0] > 0)) { //if a left wall is near
        lastWall = false;
        return WALL_LEFT;
    }

    if ((USVals[1] <= 10) && (USVals[1] > 0)) { //if a right wall is near
        lastWall = true;
        return WALL_RIGHT;
    }

    if (USVals[0] <= 20) {
        if (USVals[0] > 13) { //if a left wall is near
            return NO_WALLS_LEFT;
        }
        return FORWARD;
    }

    if (USVals[1] <= 20) {
        if (USVals[1] > 13) { //if a right wall is near
            return NO_WALLS_RIGHT;
        }
        return FORWARD;
    }


    if ((USVals[0] > 30) && (USVals[1] > 30) && (USVals[2] > 30)) {
        return NO_WALLS;
    }

    return WALL_TEST;
}



//sweeps ~175 degrees back then forward
bool Robot::wallSweep(bool dir) {
    if (dir) {
        switch (wallSweepState) {
            case SWEEP_FORWARDS:
                tempAngle = 180;
                if (turn5DegLeft(36)) {
                    wallSweepState = SWEEP_BACKWARDS;
                    //reset encoder values
                    curLTicks = lEncode;
                    curRTicks = rEncode;
                    tickLDiff = 0;
                    tickRDiff = 0;
                    rotato(1);
                }
                break;

            case SWEEP_BACKWARDS:
                tempAngle = 180;
                if (turn5DegRight(36)) {
                    wallSweepState = SWEEP_FORWARDS;
                    orange.debugLEDON();
                    //reset encoder values
                    curLTicks = lEncode;
                    curRTicks = rEncode;
                    tickLDiff = 0;
                    tickRDiff = 0;
                    return true;
                }
                break;
        }
    } else {
        switch (wallSweepState) {
            case SWEEP_FORWARDS:
                tempAngle = 180;
                if (turn5DegRight(36)) {
                    wallSweepState = SWEEP_BACKWARDS;
                    //reset encoder values
                    curLTicks = lEncode;
                    curRTicks = rEncode;
                    tickLDiff = 0;
                    tickRDiff = 0;
                    rotato(1);
                }
                break;

            case SWEEP_BACKWARDS:
                tempAngle = 180;
                if (turn5DegLeft(36)) {
                    wallSweepState = SWEEP_FORWARDS;
                    orange.debugLEDON();
                    //reset encoder values
                    curLTicks = lEncode;
                    curRTicks = rEncode;
                    tickLDiff = 0;
                    tickRDiff = 0;
                    return true;
                }
                break;
        }
    }
    return false;
}


//goes around a wall. Used in wallNav function
bool Robot::aroundWall(void) {
    switch (aroundState) {
        case LEAVE_WALL:
            if (rotato(6)) {
                aroundState = TURN_AROUND_WALL;
            }
            break;

        case TURN_AROUND_WALL:
            tempAngle = 90;
            if (lastWall) {
                if (turn5DegLeft(18)) {
                    aroundState = CATCH_WALL;
                }
            } else {
                if (turn5DegRight(18)) {
                    aroundState = CATCH_WALL;
                }
            }
            break;

        case CATCH_WALL:
            if (rotato(27)) {
                aroundState = LEAVE_WALL;
                return true;
            }
            break;
    }
    return false;
}
