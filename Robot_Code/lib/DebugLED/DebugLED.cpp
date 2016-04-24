/* Debug LED to turn on if for testing purposes
 *
 * Pins
 * Debug LED - 1 Digital I/O
 *
 * Created on Apr 20. 2016 by Ben Titus
 * Last edit made Apr 20, 2016 by Ben Titus
 */

#include "DebugLED.h"

DebugLED::DebugLED(uint8_t LED) {
    LEDPin = LED;
    pinMode(LEDPin, OUTPUT);
}


//turns the debug LED on
void DebugLED::debugLEDON(void) {
    state = true;
    digitalWrite(LEDPin, !state);
}


//turns the debug LED off
void DebugLED::debugLEDOFF(void) {
    state = false;
    digitalWrite(LEDPin, !state);
}


//toggles the debug LED state
void DebugLED::debugLEDTOG(void) {
    digitalWrite(LEDPin, state);
    state = !state;
}


//flashes the debug LED
void DebugLED::debugLEDFlash(void) {
    digitalWrite(LEDPin, 0);
    state = 0;
    digitalWrite(LEDPin, !state);
    state = !state;
    delay(150);
    digitalWrite(LEDPin, !state);
    state = !state;
}
