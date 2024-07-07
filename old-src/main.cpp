//
// Created by DJ on 27/05/2024.
//
#include <Arduino.h>
#include "thermistor.h"
#include "LiquidCrystal_I2C.h"

#define TEMP_PIN PIN_A0
#define HEAT_PIN PIND5
#define ENCODER_PIN_A 3 // must be an interrupt pin
#define ENCODER_PIN_B 7
#define ENCODER_PIN_BUTTON 2 // must be interrupt pin
#define DISPLAY_I2C_ADDR 0x3F
//
//#define LONG_PRESS_THRESHOLD 1000
//thermistor plateTemp(TEMP_PIN, 1);
//LiquidCrystal_I2C lcd(DISPLAY_I2C_ADDR, 16, 2);
//
//enum ButtonState {
//    SHORT_PRESS,
//    LONG_PRESS,
//    RELEASED
//} lastButtonState = RELEASED;

// ============================================= Profiles =============================================
// Reflow profile
String states[4] = {"Preheat      ","Soak   ","Ramp up","Reflow "};

int Times_profile[3][4] = { {60, 150, 60, 20},
                            {60, 150, 60, 20},
                            {60, 200, 60, 20}};
int Temps_profile[3][4] = { {150, 165, 235, 235},
                            {150, 165, 250, 250},
                            {150, 165, 230, 230}};
// ===========================================   Settings   ===========================================
const int timeDelay = 20;
const int overshoot = 20;
const int hotspotCompensation = 20;
const int hysteresis = 1;
// ===========================================   Variables   ===========================================
int setpoint;
int roomTemp;
int currentState;
long timeElapsed = 0;
long timeSinceLastHeat = 0;
int timeleft = 0;
int totaltimeleft = 0;
long timeHeating = 0;
long error;
float measuredTemp;
unsigned int profile = 1;
// ============================================   States   =============================================
enum reflowStates{
    HEATING,
    COOL,
    STARTUP
};
reflowStates reflowState = STARTUP;
// =============================================   Pins   =============================================
// ThermoCouple
//const int thermo_gnd_pin = A4;
//const int thermo_vcc_pin = A3;
//// Relay
//const int relay_pin = A5;
//// Display
//const int rw = 4;
//const int ledp = 10;
//const int ledm = 11;

// Buttons
const int button = 13;
// =======================================   Library objects  =========================================
//MAX6675 thermocouple(thermo_sck_pin, thermo_cs_pin, thermo_so_pin);
LiquidCrystal_I2C lcd(DISPLAY_I2C_ADDR, 16, 2);
thermistor thermocouple(TEMP_PIN, 0);
// ============================================= Setup =============================================
void setup() {
    Serial.begin(9600);
    delay(100);
    Serial.println("i, setpoint, measured, control, integral, derivative");

    // Compensate for hot spots & Add up the times
    for (uint16_t j = 0; j < sizeof(Temps_profile)/sizeof(Temps_profile[0]); j++){
        for (uint16_t i = 0; i < sizeof(Temps_profile[0])/sizeof(Temps_profile[0][0]); i++){
            Temps_profile[j][i] += hotspotCompensation;
        }
        for (uint16_t i = 1; i < sizeof(Times_profile[0])/sizeof(Times_profile[0][0]); i++){
            Times_profile[j][i] += Times_profile[j][i-1];
        }
    }

    // Pins
    pinMode(HEAT_PIN, OUTPUT);
    digitalWrite(HEAT_PIN, LOW);

    pinMode(ENCODER_PIN_BUTTON, INPUT);

    // LCD
    lcd.begin(16, 2);
    while (true) {
        lcd.setCursor(0, 1);
        lcd.print(thermocouple.analog2temp());
        lcd.setCursor(4, 1);
        lcd.print((char) 223);
        lcd.print("C");
        delay(100);
    }
}

// ============================================= Loop =============================================
void loop() {
    switch (reflowState) {
        case HEATING:{
            setpoint = Times_profile[profile][currentState];
            timeleft = Times_profile[profile][currentState] - timeElapsed;
            totaltimeleft = Times_profile[profile][3] - timeElapsed;
            // LCD
            lcd.setCursor(0,0);
            lcd.print(states[currentState]);
            lcd.setCursor(9,0);
            lcd.print("        ");
            lcd.setCursor(9,0);
            lcd.print(timeleft);
            lcd.setCursor(13,0);
            lcd.print(totaltimeleft);

            if (timeElapsed >= Times_profile[profile][currentState]){
                currentState++;
                if (currentState == 4){
                    reflowState = COOL;
                    break;
                }
            }

            if (digitalRead(ENCODER_PIN_BUTTON) == 1){
                delay(200);
                if (digitalRead(ENCODER_PIN_BUTTON) == 1){
                    reflowState = STARTUP;
                    lcd.setCursor(0,0);
                    lcd.print("Reflow stopped  ");
                    Serial.println("Reflow Stopped");
                    delay(2000);
                }
            }
        }
            break;
        case COOL:{
            setpoint = 0;
            // LCD
            lcd.setCursor(0,0);
            lcd.print("Cooling down    ");
            if(measuredTemp < 50){
                reflowState = STARTUP;
            }
        }
            break;
        case STARTUP:{
            setpoint = 0;
            // LCD
            lcd.setCursor(0,0);
            lcd.print("Profile ");
            lcd.print(profile);
            lcd.print("     ");
            lcd.setCursor(11, 1);
            lcd.print("START");
            // Check button
            if (digitalRead(ENCODER_PIN_BUTTON) == 1){
                delay(1000);
                if (digitalRead(ENCODER_PIN_BUTTON) == 1){ // long press: start reflow
                    lcd.setCursor(11, 1);
                    lcd.print(" STOP");
                    lcd.setCursor(0, 0);
                    lcd.print("Starting reflow");
                    reflowState = HEATING;
                    timeElapsed = 0;
                    currentState = 0;
                    delay(1000);
                    Serial.println("Starting reflow");
                }
                else{ // short press: switch profile
                    profile++;
                    if(profile > sizeof(Temps_profile)/sizeof(Temps_profile[0])){
                        profile = 1;
                    }
                    lcd.setCursor(0,0);
                    lcd.print("Profile ");
                    lcd.print(profile);
                    lcd.print("     ");
                    Serial.print("Switched to profile: ");
                    Serial.println(profile);
                }
            }
        }
            break;
    }

    // MEASURE CURRENT TEMPERATURE
    measuredTemp = thermocouple.analog2temp();
    lcd.setCursor(0, 1);
    lcd.print(measuredTemp);

    // CALCULATE ERROR
    error = setpoint - measuredTemp;

    // CONVERT TO BANGBANG CONTROL WITH HYSTERESIS
    if (error >= overshoot) {
        digitalWrite(HEAT_PIN, HIGH);
        timeSinceLastHeat = 0;
    }
    else {
        timeSinceLastHeat++;
        digitalWrite(HEAT_PIN, LOW);
    }

    if ((timeSinceLastHeat > 30) && (error > hysteresis)) {
        digitalWrite(HEAT_PIN, HIGH);
        timeHeating ++;
        if (timeHeating > 10) {
            timeSinceLastHeat = 0;
            timeHeating = 0;
        }
    }

    // PRINT
    Serial.print(timeElapsed);
    Serial.print(", ");
    Serial.print(setpoint);
    Serial.print(", ");
    Serial.print(measuredTemp);
    Serial.print(", ");
    Serial.print(digitalRead(HEAT_PIN) * 100);
    Serial.print(", ");
    Serial.println(error);

    // ADVANCE CLOCK
    timeElapsed++;
    if (reflowState == STARTUP){
        delay(100); // check button in startup
    }
    else{
        delay(1000);// 1s time step otherwise
    }
}


//
//
//void showSplashScreen();
//
//void encoderISR() {
//}
//
//void buttonISR() {
//    // read button
//    // don't need to debounce, the input is already debounced via a hardware filter
//    static unsigned long lastButtonPress = 0;
//    bool buttonHeld = digitalRead(ENCODER_PIN_BUTTON);
//
//    if (buttonHeld) {
//        lastButtonPress = millis();
//        // button pressed, start timer
//
//    } else if (!buttonHeld && lastButtonPress) {
//        // button released, was it short or long?
//        if (millis() - lastButtonPress < LONG_PRESS_THRESHOLD) {
//            // short press
//            lastButtonState = SHORT_PRESS;
//        } else {
//            // long press
//            lastButtonState = LONG_PRESS;
//        }
//    }
//
//}
//
//ButtonState getButtonState() {
//    auto lastState = lastButtonState;
//    lastButtonState = RELEASED;
//    return lastState;
//}
//
//
//void setup() {
//    Serial.begin(115200);
//    while (!Serial); // Wait until Serial is ready
//    // Init heater pin
//    pinMode(HEAT_PIN, OUTPUT);  // digitalPinHasPWM(p)   ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11)
//    digitalWrite(HEAT_PIN, LOW); // start with heater off
//
//    // Init encoder + button
//    pinMode(ENCODER_PIN_A, INPUT); // encoder has its own pull-downs
//    pinMode(ENCODER_PIN_B, INPUT);
//    pinMode(ENCODER_PIN_BUTTON, INPUT);
//    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);
//    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_BUTTON), buttonISR, CHANGE); // button is active high
//
//    // Start LCD
////    lcd.begin(16, 2, LCD_5x8DOTS);
//    lcd.init();
//    showSplashScreen();
//
//}
//
//void showSplashScreen() {
//    lcd.backlight();
//    lcd.setContrast(100);
//    lcd.setCursor(0, 0);
//    lcd.print("Soldering Station");
//    lcd.setCursor(0, 1);
//    lcd.print("v1.0");
//    delay(2000);
//    lcd.clear();
//}
//
//void loop() {
//    // Read temperature
//    float temp = plateTemp.analog2temp();
//    lcd.print("Temp: "); lcd.println(temp);
//    auto btn = getButtonState();
//    auto str = btn == SHORT_PRESS ? "Short press" : btn == LONG_PRESS ? "Long press" : "Released";
//    Serial.println(str);
//    delay(1000);
//}
