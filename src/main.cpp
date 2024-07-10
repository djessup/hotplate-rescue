//
// Created by DJ on 27/06/2024.
//

#if DEBUG
    #include "avr8-stub.h"
    #include "app_api.h"
#endif

#include "main.h"
#include "metric.h"
#include "pid.h"
#include "reflow_profile.h"

#include <Arduino.h>
#include <rotary.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RunningAverage.h>
#include <TaskScheduler.h>


// Next:
// - implement tasks to ramp up to temp for soak/reflow
// Last:
// - watchdog check for thermal runaway

// void callback();
// Scheduler scheduler;
// Task soakTask((TASK_SECOND * 1), TASK_FOREVER, callback);

LiquidCrystal_I2C lcd(I2C_ADDR, 16, 2);

// Global variables
unsigned int setTemp = 150;
int currentTemp = 0;
bool heatingEnabled = false;
volatile unsigned long lastTime = 0;
double errSum = 0, lastErr = 0;

volatile unsigned int encoderValue;

Rotary encoder(ENCODER_PIN_CLK, ENCODER_PIN_DT);
RunningAverage temp(TEMP_SAMPLES);
State currentState = IDLE;

State advanceState(State _state) {
    switch (_state) {
        case IDLE:
            return SOAK;

        case SOAK:
            return REFLOW;

        case REFLOW:
            return COOLDOWN;

        case COOLDOWN:
            return IDLE;
    
        default:
            return IDLE;
            break;
    }
}

/**
 * Handler for pin change interrupts for D0 to D7
 */
ISR(PCINT2_vect) 
{ 
    // Holds the pin states from the last interrupt
    static uint8_t prevPINDState = 0b00000000;
    
    static unsigned long last_exec = 0;  // for debouncing
    if ((millis() - last_exec) < 1) {
        return;
    }
    last_exec = millis();
    
    uint8_t changedBits = PIND ^ prevPINDState; // XOR to find changed bits
    prevPINDState = PIND; // Update for next time
    
    // D3 and/or D4 (PCINT19 and PCINT20) have changed
    if (changedBits & (bit(PCINT19) | bit(PCINT20))) {   
        checkEncoder();
    }
} // end of PCINT2_vect


void setup() {
    #if DEBUG
    debug_init();
    #elif SERIAL_TELEMETRY
    Serial.begin(115200);  // Initialize serial communication
    #endif

    pinMode(HEATER_PIN, OUTPUT);
    digitalWrite(HEATER_PIN, LOW);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(THERMISTOR_PIN, INPUT);
    // handled by Rotary.cpp already
    pinMode(ENCODER_PIN_CLK, INPUT);
    pinMode(ENCODER_PIN_DT, INPUT);

    Wire.begin();  // Initialize I2C communication
    delay(500);  // Wait for things to settle

    // Start with heater off
    lastTime = millis();
    encoderValue = 150;

    enablePinChangeInterrupt(ENCODER_PIN_CLK);
    enablePinChangeInterrupt(ENCODER_PIN_DT);

    initDisplay();

    temp.fillValue(readTemperature(), TEMP_SAMPLES);
}

void loop() {
    // 
   encoderValue = constrain(encoderValue, 0, 300);
   if (encoderValue != setTemp) {
       setTemp = encoderValue;
   }

    if (getButtonState() == LONG_PRESS) {
        currentState = advanceState(currentState);
    }

    switch (currentState) {
        case SOAK:
        case REFLOW:
            analogWrite(HEATER_PIN, calculatePID(setTemp, temp.getFastAverage()));
            break;

        case COOLDOWN:
        default:
            digitalWrite(HEATER_PIN, LOW);
            break;
    }

    // Read temperature
    temp.addValue(readTemperature());

    updateDisplay();

    // Telemetry
    _PM(Metric("target", setTemp));
    _PM(Metric("temp", temp.getFastAverage()));

    delay(10);
}

/**
 * TEMP
 */

float readTemperature() {
    int adcValue = analogRead(THERMISTOR_PIN); // Read the analog value
    float resistance = SERIES_RESISTOR / (1023.0 / adcValue - 1); // Calculate resistance

    // Calculate temperature using the Steinhart-Hart equation
    float steinhart;
    steinhart = resistance / THERMISTOR_NOMINAL; // (R/Ro)
    steinhart = log(steinhart); // ln(R/Ro)
    steinhart /= B_COEFFICIENT; // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; // Invert
    steinhart -= 273.15; // Convert to Celsius

    return steinhart;
}


/**
 * LCD DISPLAY 
 */

/**
 * Starts the display and prints the static portions
 */
void initDisplay() {
    lcd.begin(16, 2);
    lcd.backlight();

    lcd.setCursor(0, 0);
    lcd.print("Set:");
    lcd.setCursor(0, 1);
    lcd.print("Cur:");
} // end of initDisplay

void updateDisplay() {
    static uint8_t lastSetTemp;
    static float lastTemp;
    size_t setBuffSize = 6, tempBuffSize = 8;

    if (lastSetTemp != setTemp) {
        char setBuff[setBuffSize] = {};
        lastSetTemp = setTemp;
        itoa(setTemp, setBuff, 10); // convert to base-10 string
        size_t setDigits = strlen(setBuff);
        setBuff[setDigits] = 'C';
        // Fill empty elements with spaces
        for (size_t i = setDigits + 1; i < setBuffSize - 1; ++i) {
            if (setBuff[i] == '\0') {
                setBuff[i] = ' ';
            }
        }
        lcd.setCursor(5, 0);
        lcd.print(setBuff);
    }

    if (lastTemp != temp.getFastAverage()) {
        lastTemp = temp.getFastAverage();
        char tempBuff[tempBuffSize] = { };
        dtostrf(lastTemp, 5, 1, tempBuff); // convert to xxx.yy string
        
        lcd.setCursor(5, 1);
        lcd.print(tempBuff);
        lcd.print("C");
    }

    lcd.setCursor(12, 0);
    char stateStr[5] = {};
    switch (currentState) {
        case IDLE:
            strcpy(stateStr, "IDLE");
            break;
        case SOAK:
            strcpy(stateStr, "SOAK");
            break;
        case REFLOW:
            strcpy(stateStr, "FLOW");
            break;
        case COOLDOWN:
            strcpy(stateStr, "COOL");
            break;
        default:
            strcpy(stateStr, "UHOH");
            break;
    }
    lcd.print(stateStr);
} // end of updateDisplay



/**
 * Updates the Encoder state machine and checks for rotation events
 */
void checkEncoder() {
    unsigned char state = encoder.process();
    if (state == DIR_CW && encoderValue < TEMP_MAX) {
        encoderValue++;
    } else if (state == DIR_CCW && encoderValue > TEMP_MIN) {
        encoderValue--;
    }
} // end of checkEncoder


void enablePinChangeInterrupt(uint8_t pin) {
    // uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *pcmsk = digitalPinToPCMSK(pin);
    uint8_t PCICRbit = digitalPinToPCICRbit(pin);
    uint8_t PCMSKbit = digitalPinToPCMSKbit(pin);

    // Enable the pin change interrupt for the specific pin
    *pcmsk |= bit(PCMSKbit);

    // Clear any outstanding interrupts
    PCIFR |= bit(PCICRbit);

    // Enable pin change interrupts for the port
    PCICR |= bit(PCICRbit);
}

/**
 * Returns the state of the button or button presses
 */
ButtonResult getButtonState() {
    // If pressStart > 0 the button was released since the last check
    static unsigned long buttonPressStart = 0, timeHeld = 0;
    static bool newPress = true;

    unsigned long now = millis();
    ButtonResult result = RELEASED;
    
    if (digitalRead(BUTTON_PIN)) { // button held
        if (buttonPressStart == 0) { // is this the start of the press?
            buttonPressStart = now;
        }
        
        timeHeld = now - buttonPressStart; // Update time held
        
        if (timeHeld >= BUTTON_LONG_PRESS_MILLIS && newPress) { // long-press acheivement unlocked!
            result = LONG_PRESS;
            newPress = false;
        } else if (timeHeld >= BUTTON_SHORT_PRESS_MILLIS) { // basically a debounce...
            result = HELD;
        }

    } else { // button released
        result = (timeHeld >= BUTTON_SHORT_PRESS_MILLIS && newPress) ? SHORT_PRESS : RELEASED;

        // Start a new press sequence
        timeHeld = 0;
        buttonPressStart = 0;
        newPress = true;
    }

    // Telemetry
    _PM(Metric("button.pressStart", buttonPressStart));
    _PM(Metric("button.newPress", newPress));
    _PM(Metric("button.timeHeld", timeHeld));
    _PM(Metric("button.result", result));

    return result;
} // end of getButtonState