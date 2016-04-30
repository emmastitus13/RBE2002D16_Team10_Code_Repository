#include "Turn.h"


Turn::Turn(DriveTrain& rb):
	robotDrive(rb){

}


bool Turn::turnSlightLeft(void) {
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
bool Turn::turnSlightRight(void) {
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
