#ifndef GYRO_H
#define GYRO_H

#include "L3G.h"
#include <Wire.h>

extern volatile unsigned int timer;

extern bool gyroGood;
extern float G_Dt;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

extern long gyroTimer;   //general purpose timer
extern long gyroTimer1;
extern float G_gain; // gyros gain factor for 250deg/se
extern float gyro_z; //gyro x val
extern float gyro_zold; //gyro cummulative z value
extern float gerrz; // Gyro 7 error
extern float minErr;

class Gyro{
  public:
    Gyro();
    bool gyroSetup();
    void gyroRead();

  private:
    L3G gyro;
};

#endif
