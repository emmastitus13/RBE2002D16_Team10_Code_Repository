/* LCD controller for a 16x2 LCD screen
 *
 * Pins
 *
 *
 *
 *
 * Created on Apr 13, 2016 by Ben Titus
 * Last edit made Apr 17, 2016 by Ben Titus
 */


#ifndef LCDController_h
#define LCDController_h

#include <Arduino.h>
#include <LiquidCrystal.h>

class LCDControler {
    public:
        LCDControler(/*pins*/); //constructor
        void updatePosition(/*X,Y,Z variables*/); //updates the X, Y, and Z position of the candle
        void clearScreen(void); //clears the screen
        //feel free to add more

    private:
        //variables for each pin used and maybe the LCD variable
};

#endif
