#include <Arduino.h>

// Pin definitions
#define HEATER_PIN 9
#define THERMISTOR_PIN A6
#define BUTTON_PIN 2
#define ENCODER_PIN_CLK 3
#define ENCODER_PIN_DT 4

// LCD setup
//#define I2C_ADDR 0x27
#define I2C_ADDR 0x3F // Freenove 16x2 LCD

// Minimum time the button has to be HIGH to trigger
#define BUTTON_MIN_PRESS_MILLIS 700

// Thermistor constants
#define THERMISTOR_NOMINAL (100000) // 100k
#define TEMPERATURE_NOMINAL (25)
#define B_COEFFICIENT (3950)
#define SERIES_RESISTOR (4700) // 10k
#define TEMP_SAMPLES 50 // number of samples to average
#define TEMP_MAX (300)
#define TEMP_MIN (0)

// PID constants
#define Kp (60.00f)
#define Ki (1.00f)
#define Kd (600.00f)

enum State {
    IDLE,
    SOAK,
    REFLOW,
    COOLDOWN,
};


float readTemperature();
unsigned int calculatePID(float target, float input);
void updateDisplay();
void enablePinChangeInterrupt(uint8_t pin);
void checkEncoder();