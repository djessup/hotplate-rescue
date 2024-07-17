#ifndef HOTPLATE_RESCUE_MAIN_H
#define HOTPLATE_RESCUE_MAIN_H

#include <Arduino.h>

// Telemetry
#ifndef SERIAL_TELEMETRY
#define SERIAL_TELEMETRY 0
#endif // SERIAL_TELEMETRY

// Pin definitions
#define HEATER_PIN 9
#define THERMISTOR_PIN A6
#define BUTTON_PIN 2
#define ENCODER_PIN_CLK 3
#define ENCODER_PIN_DT 4

// LCD setup
#define I2C_ADDR 0x3F // Freenove 16x2 LCD

// Thermistor constants
#define THERMISTOR_NOMINAL (100000) // 100k
#define TEMPERATURE_NOMINAL (25)
#define B_COEFFICIENT (3950)
#define SERIES_RESISTOR (4700) // 10k
#define TEMP_SAMPLES 50 // number of samples to average
#define TEMP_MAX (250)
#define TEMP_MIN (25)

/**
 * STATE MACHINE
 */
enum State {
  IDLE,
  SOAK,
  REFLOW,
  COOLDOWN,
};
State advanceState(State _state);

/**
 * BUTTON
 */
// Minimum time the button must be held for press to register
#define BUTTON_SHORT_PRESS_MILLIS 50
#define BUTTON_LONG_PRESS_MILLIS 700

enum ButtonResult {
  RELEASED = 0b00, // button is not being held, no new presses seen
  HELD = 0b01, // button is being held down, but not long enough to register a press
  SHORT_PRESS = 0b10, // button was short-pressed and released
  LONG_PRESS = 0b11, // button was long-pressed (may still be held)
};

ButtonResult getButtonState();

void enablePinChangeInterrupt(uint8_t pin);
void checkEncoder();
float readTemperature();

/**
 * DISPLAY
 */
void printDisplayFurniture();
void updateDisplay();

#endif //HOTPLATE_RESCUE_MAIN_H
