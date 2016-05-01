#include "Robot.h"


Robot::Robot(LiquidCrystal& lcd, FireExtinguisher& fx, DriveTrain& rb, NewPing& lus, NewPing& rus, NewPing& fus, DebugLED& _orange, DebugLED& _blue, Gyro& _gyro):
	LCD(lcd)
	,fireExtinguisher(fx)
	,robotDrive(rb)
	,leftUS(lus),rightUS(rus),frontUS(fus)
	,orange(_orange),blue(_blue)
	,gyro(_gyro){

}


//prints the X, Y, and Z positions of the candle
void  Robot::updatePosition(float newX, float newY, float newZ) {
	int x = newX + 0.5;
	int y = newY + 0.5;
	int z = newZ + 0.5;
    char pos[16];
    sprintf(pos, "X %2i,Y %2i,Z %2i", x, y, z);
    LCD.setCursor(0,0);
    LCD.print(pos);
}

//calculates the position of the candle then stores it in xPos, yPos, and zPos
//this function will be called after extinguishing the candle
void Robot::calcDist(Movement mov) {
    float mag;
	mag = ((mov.encTicks / TICK_PER_WHEEL_REV) * WHEEL_CIRC * 3.0/2.0) / 25.4;

    xPos += mag * cos(mov.angle * PI / 180);
    yPos += mag * sin(mov.angle * PI / 180);
}


//calculates and updates the current position
void Robot::logMove(Movement mov) {
	calcDist(movBuf);
	updatePosition(xPos, yPos, zPos);
}


//calculates the height of the candle
void Robot::candleZ(void) {
    readAllUS();
    float angle;
	angle = fireExtinguisher.servoPosToAngle() * PI / 180;
    zPos = (((USVals[2] + 4.0)) * tan(angle) + 18.0) / 2.54;
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

}//sweeps back and forth to find the candle
bool Robot::candleFind(void) {

    switch(candleState) {
        case CANDLE_FIND:
            fireExtinguisher.servoTilt(60);
            LCD.setCursor(0,1);
            LCD.print("FIND            ");
            candleState = candleTest();
            break;

        case CANDLE_NOT_FOUND:
            LCD.setCursor(0,1);
            LCD.print("No find");

            angAttempt = 360;
            turn5DegRight(80);
            if (!fireExtinguisher.readFlameSenseDig()) {
				movBuf.angle = tempAngle;
                robotDrive.botStop();
	            //reset encoder values
	            curLTicks = lEncode;
	            curRTicks = rEncode;
	            tickLDiff = 0;
	            tickRDiff = 0;
	            //reset the gyro
	            gyroGood = false;
	            gyro_z = 0;
	            gyro_zold = 0;

            }
            candleState = candleTest();
            break;

        case CANDLE_FOUND:
            LCD.setCursor(0,1);
            LCD.print("Found           ");
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
            fireExtinguisher.servoTilt(servoIndex + servoMinimum);
            delay(100);
            LCD.setCursor(0,1);
            LCD.print("Extinguishing   ");
            fireExtinguisher.extinguishFire();
            if (fireExtinguisher.readFlameSense() > 980) {
				candleZ();
				movBuf.encTicks = 2 * TICK_PER_WHEEL_REV;
				calcDist(movBuf);
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
                LCD.setCursor(0,1);
                LCD.print("Flame out   ");
                drawn = true;
                return true;
            }
            break;

        default:
            candleState = CANDLE_FIND;
            break;
    }
    return false;
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


//changes the state of maze navigation based on the sensors
uint8_t Robot::mazeWallTest(void) {
    fireExtinguisher.servoTilt(60);
    readAllUS();
    if (((USVals[2] > 0) && (USVals[2] < 25)) ||
            ((USVals[0] > 0) && (USVals[0] < 25)) ||
            ((USVals[1] > 0) && (USVals[1] < 25))) { //if there is a wall near
                return WALL_AVOID;
    }
	if (mazeState == WALL_AVOID) {
		return SCAN_FOR_FIRE;
	} else if (mazeState == STEP_FORWARD) {
        return STEP_FORWARD;
    }
}


//navigates through the maze
bool Robot::mazeSearch(void) {
    switch (mazeState) {
        case MAZE_TEST:
            robotDrive.botStop();
            LCD.setCursor(0,1);
            LCD.print("TESTING         ");
            mazeState = mazeWallTest();
            break;

        case WALL_AVOID:
            LCD.setCursor(0,1);
            LCD.print("Get out         ");
            if (wallNav()) {
                mazeState = SCAN_FOR_FIRE;
            }
            break;

        case SCAN_FOR_FIRE:
            LCD.setCursor(0,1);
            LCD.print("Scanning        ");
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
            LCD.setCursor(0,1);
            LCD.print("Forward         ");
            mazeState = mazeWallTest();
            if (rotato(12)) {
                mazeState = MAZE_TEST;
            }
			break;

        case FIRE_DETECTED:
            robotDrive.botStop();
            LCD.setCursor(0,1);
            LCD.print("Fire detected   ");
            return true;
            break;
    }
    return false;
}


//moves the robot forward one-sixth of a wheel rotation
bool Robot::rotato(uint8_t sixths) {

    tickRDiff = (rEncode - curRTicks);
    tickLDiff = (lEncode - curLTicks);

    if((tickLDiff <= (TICK_PER_SIXTH_WHEEL_REV*sixths)) || (tickRDiff <= (TICK_PER_SIXTH_WHEEL_REV*sixths))){
        driveStraight();
        return false;
    } else {
        robotDrive.botStop();
        movBuf.encTicks = min(rEncode, lEncode) - min(curLTicks, curRTicks);
		logMove(movBuf);
		orange.debugLEDON();
		orange.debugLEDOFF();
        //reset encoder values
        curLTicks = lEncode;
        curRTicks = rEncode;
        tickLDiff = 0;
        tickRDiff = 0;
    }
	return true;
}

//drives the robot straight according to the encoders
void Robot::driveStraight() {
    robotDrive.botDrive(driveL, driveR);
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

            if ((tickRDiff < (TICK_PER_5_DEG*deg5)) || (tickLDiff < (TICK_PER_5_DEG*deg5))) {
                robotDrive.botDrive(100, -100);
                return false;
            }
            robotDrive.botStop();
            movBuf.angle += angAttempt;
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
                tempAngle = angAttempt * gyro_z / (IMU_TURN_5_LEFT * deg5);
                return false;
            }
            robotDrive.botStop();
            movBuf.angle += angAttempt;
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

            if ((tickRDiff < (TICK_PER_5_DEG*deg5)) || (tickLDiff < (TICK_PER_5_DEG*deg5))) {
                robotDrive.botDrive(-100, 100);
                return false;
            }
            robotDrive.botStop();
            movBuf.angle += angAttempt;
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
                tempAngle = angAttempt * gyro_z / (IMU_TURN_5_LEFT * deg5); //tempAngle = % turned scaled by attempted angle
                return false;
            }
            robotDrive.botStop();
            movBuf.angle += angAttempt;
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
            angAttempt = 90;
            if (turn5DegLeft(11)) {
                wallState = WALL_TEST;
            }
            break;

        case TURN_RIGHT:
            angAttempt = 90;
            if (turn5DegRight(11)) {
                wallState = WALL_TEST;
            }
            break;

        case WALL_RIGHT:
            orange.debugLEDON();
			driveL = baseDrive;
			driveR = baseDrive + 5;
	            if (rotato(9)) {
	                wallState = WALL_TEST;
	            }
            break;

        case WALL_LEFT:
            blue.debugLEDON();
			driveL = baseDrive;
			driveR = baseDrive - 15;
	            if (rotato(9)) {
	                wallState = WALL_TEST;
	            }
            break;

        case FORWARD:
			driveL = baseDrive;
			driveR = baseDrive - 3;
            if (rotato(9)) {
                wallState = WALL_TEST;
            }
            break;

        case WALL_SCAN_RIGHT:
            wallCount = 0;
            if (wallSweep(0)) {
                wallState = TURN_RIGHT;
            }
            if (!fireExtinguisher.readFlameSenseDig()) {
				movBuf.angle = tempAngle;
                robotDrive.botStop();
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
            }
            break;

        case WALL_SCAN_LEFT:
            wallCount = 0;
            if (wallSweep(1)) {
                wallState = TURN_LEFT;
            }
            if (!fireExtinguisher.readFlameSenseDig()) {
				movBuf.angle = tempAngle;
                robotDrive.botStop();
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
            }
            break;

        case NO_WALLS:
            if (aroundWall()) {
                wallState = WALL_TEST;
            }
            break;

        case NO_WALLS_RIGHT:
			driveL = baseDrive;
			driveR = baseDrive - 15;
            wallState = FORWARD;
            break;

        case NO_WALLS_LEFT:
			driveL = baseDrive;
			driveR = baseDrive + 3;
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

    if ((USVals[2] < 24) && (USVals[2] > 0)) { //if a wall in front
        if ((USVals[0] > USVals[1]) && (USVals[1] > 0)) { //if a left wall is nearer than a right wall
                return WALL_SCAN_RIGHT;
        } else { //if a right wall is nearer than a left wall or no wall is nearer
                return WALL_SCAN_LEFT;
        }
    }

    if ((USVals[0] < 13) && (USVals[0] > 0)) { //if a left wall is near
        lastWall = false;
        return WALL_LEFT;
    }

    if ((USVals[1] < 13) && (USVals[1] > 0)) { //if a right wall is near
        lastWall = true;
        return WALL_RIGHT;
    }


    if ((USVals[0] > 40) && (USVals[1] > 40) && (USVals[2] > 40)) {
        return NO_WALLS;
    }

    if (USVals[0] < 40) {
        if (USVals[0] > 17) { //if a left wall is near
            return NO_WALLS_LEFT;
        }
        return FORWARD;
    }

    if (USVals[1] < 40) {
        if (USVals[1] > 17) { //if a right wall is near
            return NO_WALLS_RIGHT;
        }
        return FORWARD;
    }

    return WALL_TEST;
}



//sweeps ~175 degrees back then forward
bool Robot::wallSweep(bool dir) {
    if (dir) {
        switch (wallSweepState) {
            case SWEEP_FORWARDS:
				angAttempt = 180;
                if (turn5DegLeft(30)) {
                    wallSweepState = SWEEP_BACKWARDS;
                    //reset encoder values
                    curLTicks = lEncode;
                    curRTicks = rEncode;
                    tickLDiff = 0;
                    tickRDiff = 0;
                }
                break;

            case SWEEP_BACKWARDS:
				angAttempt = -180;
                if (turn5DegRight(30)) {
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
				angAttempt = -180;
                if (turn5DegRight(30)) {
                    wallSweepState = SWEEP_BACKWARDS;
                    //reset encoder values
                    curLTicks = lEncode;
                    curRTicks = rEncode;
                    tickLDiff = 0;
                    tickRDiff = 0;
                }
                break;

            case SWEEP_BACKWARDS:
				angAttempt = 180;
                if (turn5DegLeft(30)) {
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
            angAttempt = 90;
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
