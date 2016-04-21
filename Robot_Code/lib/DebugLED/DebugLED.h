/* Debug LED to turn on if for testing purposes
 *
 * Pins
 * Debug LED - 1 Digital I/O
 *
 * Created on Apr 20. 2016 by Ben Titus
 * Last edit made Apr 20, 2016 by Ben Titus
 */

#ifndef DebugLED_h
#define DebugLED_h

#include <Arduino.h>

class DebugLED {
    public:
        DebugLED(uint8_t LED);
        void debugLEDON(void);
        void debugLEDOFF(void);
        void debugLEDTOG(void);
        void debugLEDFlash(void);

    private:
        uint8_t LEDPin;
        bool state;
};

#endif
