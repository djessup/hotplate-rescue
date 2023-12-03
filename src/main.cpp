//
// Created by DJ on 1/12/2023.
//

#include "main.h"
#include "EnableInterrupt.h"

#define PIN_TEMP PIN_A0
#define PIN_ISENSE PIN_A1
#define PIN_HEATER 3
#define PIN_BTN_UP 4
#define PIN_BTN_OK 7
#define PIN_BTN_DOWN 8

#define DEBOUNCE_DELAY 100 // in ms

volatile uint16_t interruptCnt_btnUp, interruptCnt_btnDown, interruptCnt_btnOk = 0;
void isr_btnUp() {
    interruptCnt_btnUp++;
}
void isr_btnDown() {
    interruptCnt_btnDown++;
}
void isr_btnOk() {
    interruptCnt_btnOk++;
}
bool i2CAddrTest(uint8_t addr) {
    Wire.begin();
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
        return true;
    }
    return false;
}
//uint32_t last_interrupt_time_btnUp = 0;
//void isr_btnUpDebounce() {
//    uint32_t this_interrupt_time = millis();
//
//    if (this_interrupt_time - last_interrupt_time_btnUp > DEBOUNCE_DELAY) {
//        interruptCnt_btnUp++;
//    }
//
//    last_interrupt_time_btnUp = this_interrupt_time;
//}



void setup() {
    if (!i2CAddrTest(0x27)) {
        lcd = LiquidCrystal_I2C(0x3F, 16, 2);
    }

    // Init Arduino features
    Serial.begin(115200);
    analogReference(EXTERNAL); // Connect AREF to 3.3V and use that, less noisy!

    // Init LCD
    lcd.init();
    lcd.backlight();

    // Init inputs
    pinMode(PIN_TEMP, INPUT);
    pinMode(PIN_ISENSE, INPUT);
    pinMode(PIN_BTN_UP, INPUT_PULLUP);
    pinMode(PIN_BTN_OK, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);

    // Init outputs
    pinMode(PIN_HEATER, OUTPUT);

    // Interrupts
    enableInterrupt(PIN_BTN_UP, isr_btnUp, CHANGE);
    enableInterrupt(PIN_BTN_DOWN, isr_btnDown, CHANGE);
    enableInterrupt(PIN_BTN_OK, isr_btnOk, CHANGE);
}

void readTemp() {
//100k thermistor
//100k series
    uint8_t NUMSAMPLES = 5;
    uint8_t samples[NUMSAMPLES - 1];
    long SERIESRESISTOR = 100000; // 100k

    uint8_t i;
    float average;
    // take N samples in a row, with a slight delay
    for (i = 0; NUMSAMPLES > i; i++) {
        samples[i] = analogRead(PIN_TEMP);
        delay(10);
    }

    // average all the samples out
    average = 0;
    for (i = 0; i < NUMSAMPLES; i++) {
        average += samples[i];
    }
    average /= NUMSAMPLES;

    lcd.setCursor(0,0);  // set the cursor to column 0, line 0
    lcd.print("Avg of ");lcd.print(NUMSAMPLES);lcd.print(": ");
    lcd.println(average);
    // convert the value to resistance
    average = 1023 / average - 1;
    average = SERIESRESISTOR / average;

    lcd.setCursor(0,1);  // set the cursor to column 0, line 1
    lcd.print("NTC res. ");
    lcd.println(average);
    delay(100);
//    lcd.clear(); // Clears the display
}


bool led = false;

void loop() {
    readTemp();
    led = !led;
    digitalWrite(LED_BUILTIN, led);
}
