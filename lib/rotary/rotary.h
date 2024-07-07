#ifndef HOTPLATE_RESCUE_ROTARY_H
#define HOTPLATE_RESCUE_ROTARY_H

/*
 * Rotary encoder library for Arduino.
 * Based on https://github.com/buxtronix/arduino/blob/master/libraries/Rotary/Rotary.h
 */

// Enable this to emit codes twice per step.
//#define HALF_STEP

// Enable weak pullups
// #define ENABLE_PULLUPS


// Values returned by 'process'
// No complete step yet.
#define DIR_NONE 0b00000000
// Clockwise step.
#define DIR_CW 0b00010000
// Anti-clockwise step.
#define DIR_CCW 0b00100000

class Rotary {
public:
    Rotary(uint8_t, uint8_t);
    // Process pin(s)
    uint8_t process();
    // Get the last processed state
    uint8_t getState();

private:
    uint8_t state;
    uint8_t pin1, pin2;
    uint8_t port1, port2;
    volatile uint8_t* pinReg1;
    volatile uint8_t* pinReg2;
};


#endif //HOTPLATE_RESCUE_ROTARY_H
