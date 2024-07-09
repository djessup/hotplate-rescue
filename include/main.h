#include <Arduino.h>
#ifndef SERIAL_TELEMETRY
#define SERIAL_TELEMETRY 0
#endif

struct Metric {
    String name;
    enum ValueType {
        INT,
        LONG,
        DOUBLE,
        FLOAT,
        UINT,
        ULONG,
        INVALID
    } type;
    union {
        int intValue;
        long longValue;
        double doubleValue;
        float floatValue;
        unsigned int uintValue;
        unsigned long ulongValue;
    } value;

    // Constructor for int
    Metric(const String &n, int v) : name(n), type(INT) {
        value.intValue = v;
    }

    // Constructor for long
    Metric(const String &n, long v) : name(n), type(LONG) {
        value.longValue = v;
    }

    // Constructor for double
    Metric(const String &n, double v) : name(n), type(DOUBLE) {
        value.doubleValue = v;
    }

    // Constructor for float
    Metric(const String &n, float v) : name(n), type(FLOAT) {
        value.floatValue = v;
    }

    // Constructor for unsigned int
    Metric(const String &n, unsigned int v) : name(n), type(UINT) {
        value.uintValue = v;
    }

    // Constructor for unsigned long
    Metric(const String &n, unsigned long v) : name(n), type(ULONG) {
        value.ulongValue = v;
    }
};

#if SERIAL_TELEMETRY
//===== Debugging macros (pinched from TaskScheduler examples)========================
#define SerialD Serial
// Print line w/ timestamp in millis (millis: msg)
#define _PT(a) SerialD.print(millis()); SerialD.print(": "); SerialD.println(a)
// Print message only
#define _PP(a) SerialD.print(a)
// Print message w/ newline
#define _PL(a) SerialD.println(a)
// Print message as HEX w/ newline
#define _PX(a) SerialD.println(a, HEX)
// Print a telemetry metric
void _PM(Metric metric) {
    SerialD.print(">");
    SerialD.print(metric.name); 
    SerialD.print(":");
     switch (metric.type) {
        case Metric::INT:
            Serial.println(metric.value.intValue);
            break;
        case Metric::LONG:
            Serial.println(metric.value.longValue);
            break;
        case Metric::DOUBLE:
            Serial.println(metric.value.doubleValue);
            break;
        case Metric::FLOAT:
            Serial.println(metric.value.floatValue);
            break;
        case Metric::UINT:
            Serial.println(metric.value.uintValue);
            break;
        case Metric::ULONG:
            Serial.println(metric.value.ulongValue);
            break;
        default:
            Serial.println("Invalid Type");
    }
}
#else
#define _PT(a)
#define _PP(a)
#define _PL(a)
#define _PX(a)
#define _PM(a)
#endif


// Pin definitions
#define HEATER_PIN 9
#define THERMISTOR_PIN A6
#define BUTTON_PIN 2
#define ENCODER_PIN_CLK 3
#define ENCODER_PIN_DT 4

// LCD setup
//#define I2C_ADDR 0x27
#define I2C_ADDR 0x3F // Freenove 16x2 LCD

// Thermistor constants
#define THERMISTOR_NOMINAL (100000) // 100k
#define TEMPERATURE_NOMINAL (25)
#define B_COEFFICIENT (3950)
#define SERIES_RESISTOR (4700) // 10k
#define TEMP_SAMPLES 50 // number of samples to average
#define TEMP_MAX (250)
#define TEMP_MIN (25)

// PID constants
#define Kp (80.00f)
#define Ki (3.00f)
#define Kd (150.00f)

enum State {
    IDLE,
    SOAK,
    REFLOW,
    COOLDOWN,
};



// Minimum time the button must be held for press to register
#define BUTTON_SHORT_PRESS_MILLIS 50
#define BUTTON_LONG_PRESS_MILLIS 700

/**
 * BUTTON
 */
enum ButtonResult {
    RELEASED = 0b00, // button is not being held, no new presses seen
    HELD = 0b01,  // button is being held down, but not long enough to register a press
    SHORT_PRESS = 0b10, // button was short-pressed and released 
    LONG_PRESS = 0b11, // button was long-pressed (may still be held)
};

ButtonResult getButtonState();


void checkEncoder();
float readTemperature();


unsigned int calculatePID(float target, float input);

void initDisplay();
void updateDisplay();

void enablePinChangeInterrupt(uint8_t pin);
