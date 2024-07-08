//
// Created by DJ on 27/06/2024.
//

#ifndef HOTPLATE_RESCUE_MAIN_H
#define HOTPLATE_RESCUE_MAIN_H

#include "main.h"
#include "reflow_profile.h"

#if DEBUG
    #include "avr8-stub.h"
    #include "app_api.h"
#endif

#include "Arduino.h"
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

/*
 * Pin Change Interrupts
 * D0	  PCINT16 (PCMSK2 / PCIF2 / PCIE2)
 * D1	  PCINT17 (PCMSK2 / PCIF2 / PCIE2)
 * D2	  PCINT18 (PCMSK2 / PCIF2 / PCIE2)
 * D3	  PCINT19 (PCMSK2 / PCIF2 / PCIE2)
 * D4	  PCINT20 (PCMSK2 / PCIF2 / PCIE2)
 * D5	  PCINT21 (PCMSK2 / PCIF2 / PCIE2)
 * D6	  PCINT22 (PCMSK2 / PCIF2 / PCIE2)
 * D7	  PCINT23 (PCMSK2 / PCIF2 / PCIE2)
 * D8	  PCINT0  (PCMSK0 / PCIF0 / PCIE0)
 * D9	  PCINT1  (PCMSK0 / PCIF0 / PCIE0)
 * D10	  PCINT2  (PCMSK0 / PCIF0 / PCIE0)
 * D11	  PCINT3  (PCMSK0 / PCIF0 / PCIE0)
 * D12	  PCINT4  (PCMSK0 / PCIF0 / PCIE0)
 * D13	  PCINT5  (PCMSK0 / PCIF0 / PCIE0)
 * A0	  PCINT8  (PCMSK1 / PCIF1 / PCIE1)
 * A1	  PCINT9  (PCMSK1 / PCIF1 / PCIE1)
 * A2	  PCINT10 (PCMSK1 / PCIF1 / PCIE1)
 * A3	  PCINT11 (PCMSK1 / PCIF1 / PCIE1)
 * A4	  PCINT12 (PCMSK1 / PCIF1 / PCIE1)
 * A5	  PCINT13 (PCMSK1 / PCIF1 / PCIE1)
 *
 * To handle a pin change interrupt you need to:
 *  - Specify which pin in the group. This is the PCMSKn variable (where n is 0, 1 or 2 from the table below).
 *     You can have interrupts on more than one pin.
 *  - Enable the appropriate group of interrupts (0, 1 or 2)
 *  - Supply an interrupt handler as shown above
 */
//
//ISR (PCINT0_vect)
//{
//    // handle pin change interrupt for D8 to D13 here
//}  // end of PCINT0_vect
//
//ISR (PCINT1_vect)
//{
//    // handle pin change interrupt for A0 to A5 here
//}  // end of PCINT1_vect
//
//ISR (PCINT2_vect)
//{
//    // handle pin change interrupt for D0 to D7 here
//}  // end of PCINT2_vect
//
////
////void setup ()
////{
////    // pin change interrupt (example for D9)
////    PCMSK0 |= bit (PCINT1);  // want pin 9
////    PCIFR  |= bit (PCIF0);   // clear any outstanding interrupts
////    PCICR  |= bit (PCIE0);   // enable pin change interrupts for D8 to D13
////}
//
//void ISR_encoderClk() {
//    static unsigned long last_exec = 0;  // for debouncing
//
//    if ((millis() - last_exec) < 10) {
//        return;
//    }
//
//    encoderValue = (digitalRead(ENCODER_PIN_DT) == HIGH) ?  encoderValue + 1 : encoderValue - 1;
//
//    last_exec = millis();
//    lastTime = last_exec;
//}


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

ISR(PCINT2_vect) 
{  // handle pin change interrupt for D0 to D7

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
}


void checkEncoder() {
    unsigned char state = encoder.process();
    if (state == DIR_CW && encoderValue < TEMP_MAX) {
        encoderValue++;
    } else if (state == DIR_CCW && encoderValue > TEMP_MIN) {
        encoderValue--;
    }
}
// end of PCINT2_vect


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


bool checkButton() {

    static unsigned long buttonPressStart = 0;
    unsigned long now = millis();

    if (digitalRead(BUTTON_PIN)) {
        
        if (buttonPressStart <= 0) {
            buttonPressStart = now;
        } 
        if (now - buttonPressStart >= BUTTON_MIN_PRESS_MILLIS) {
            buttonPressStart = 0;
            return true;
        }
    } else {
        buttonPressStart = 0;
    }

    return false;
}

void setup() {
    #if DEBUG
    debug_init();
    #else
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
    // Read encoder
   encoderValue = constrain(encoderValue, 0, 300);
   if (encoderValue != setTemp) {
       setTemp = encoderValue;
   }
//    Serial.println("enc, setpoint, temp, enabled");
//    Serial.print(setTemp); Serial.print(",");
//    Serial.print(currentTemp); Serial.print(",");
//    Serial.println(heatingEnabled);

    if (checkButton()) {
        currentState = advanceState(currentState);
    }

    switch (currentState) {
        case SOAK:
            
            analogWrite(HEATER_PIN, calculatePID(setTemp, temp.getFastAverage())*.75);
            break;

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

    // Update heater
//    if (heatingEnabled) {
//        int output = computePID(setTemp, currentTemp);
//        analogWrite(HEATER_PIN, output);
//    } else {
//        analogWrite(HEATER_PIN, 0);
//    }

   updateDisplay();

    // Output temperature and setpoint via serial
//    Serial.print("Setpoint: ");
//    Serial.print(setTemp);
//    Serial.print(" Current Temp: ");
//    Serial.println(currentTemp);

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
 * PID CONTROLLER
 */

unsigned int calculatePID(float setpoint, float input) {
    static float integralTerm = 0.0f;
    static float lastInput = 0.0f;
    static float derivativeFilter = 0.0f; // For filtering the derivative term
    static const float alpha = 0.1; // Smoothing factor for derivative filtering
    static unsigned long lastTime = 0;

    // Update the current time
    unsigned long currentTime = millis();
    unsigned long timeChange = (currentTime - lastTime);

    // Compute the error
    float error = setpoint - input;

    // Proportional term
    float Pout = Kp * error;

    // Integral term
    integralTerm += (Ki * error * (timeChange / 1000.0f));

    // Anti-windup: prevent integral term from getting too large
    // if (integralTerm > 255.0f) integralTerm = 255.0f;
    // else if (integralTerm < 0.0f) integralTerm = 0.0f;
    // Integral clamping
    if (integralTerm > 150.0f) integralTerm = 150.0f;
    else if (integralTerm < -150.0f) integralTerm = -150.0f;

    // Derivative term
    float derivative = (input - lastInput) / (timeChange / 1000.0f);
    // float Dout = Kd * derivative;
    // Derivative filter
    derivativeFilter = (alpha * derivative) + ((1 - alpha) * derivativeFilter);
    float Dout = Kd * derivativeFilter;

    // Compute the total output
    float output = Pout + integralTerm - Dout;

    // Restrict the output to the range 0-255
    if (output > 255.0f) output = 255.0f;
    else if (output < 0.0f) output = 0.0f;

    // Remember the last input and time for the next calculation
    lastInput = input;
    lastTime = currentTime;

    return (unsigned int)round(output);
}


/**
 * LCD DISPLAY 
 */

void initDisplay() {
    lcd.begin(16, 2);
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Set:");
    lcd.setCursor(0, 1);
    lcd.print("Cur:");
}

void updateDisplay() {
    static uint8_t lastSetTemp;
    static float lastTemp;
    uint8_t setBuffSize = 6, tempBuffSize = 8;

    if (lastSetTemp != setTemp) {
        char setBuff[setBuffSize] = {};
        lastSetTemp = setTemp;
        itoa(setTemp, setBuff, 10); // convert to base-10 string
        int setDigits = strlen(setBuff);
        setBuff[setDigits] = 'C';
        // Fill empty elements with spaces
        for (unsigned int i = setDigits + 1; i < setBuffSize - 1; ++i) {
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
}
#endif //HOTPLATE_RESCUE_MAIN_H
