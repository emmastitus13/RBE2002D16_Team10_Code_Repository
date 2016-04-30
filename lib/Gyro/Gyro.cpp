#include "Gyro.h"


Gyro::Gyro(){

}

//initializes the gyro
bool Gyro::gyroSetup() {
    delay(50);
    gyro.enableDefault(); // gyro init. default 250/deg/s
    delay(700);// allow time for gyro to settle
    for(int i =0;i<100;i++){  // takes 100 samples of the gyro
        gyro.read();
        gerrz+=gyro.g.z;
        delay(25);
    }
    gerrz = gerrz/100;
    return true;
}


//reads the value of the gyro
void Gyro::gyroRead() {
    Wire.begin();
    gyro.init();
    if (!gyroGood) {
        gyroGood = gyroSetup();
    }
    if(timer % 5) { // reads imu every 5ms
        gyro.read(); // read gyro
        gyro_z=(float)(gyro.g.z-gerrz)*G_gain;
        gyro_z = gyro_z*G_Dt;
        if (abs(gyro_z - gyro_zold) > minErr) {
            gyro_z +=gyro_zold;
        }
        gyro_zold=gyro_z ;
    }
}
