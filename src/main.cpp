//
// Created by DJ on 1/12/2023.

#include "main.h"
#include <EEPROM.h>
#include <AceButton.h>
#include "EnableInterrupt.h"

using namespace ace_button;

#define PIN_TEMP PIN_A0
#define PIN_ISENSE PIN_A1
#define PIN_HEATER 3
#define PIN_BTN_UP 4
#define PIN_BTN_OK 7
#define PIN_BTN_DOWN 8

#define DEBOUNCE_DELAY 200 // in ms

// The beta coefficient of the thermistor (usually 3000-4000)
#define BETA_COEFFICIENT 3950
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000
// series resistance (47k pull-up)
#define SERIESRESISTOR 47000

#define EEPROM_SETTINGS 0;

struct HeaterSettings {
    float targetTemp;
};

bool i2CAddrTest(uint8_t addr);
//
//volatile uint16_t interruptCnt_btnUp, interruptCnt_btnDown, interruptCnt_btnOk = 0;
//
//uint32_t last_interrupt_time_btnUp = 0;
//
//void isr_btnUpDebounce() {
//    uint32_t this_interrupt_time = millis();
//
//    if (this_interrupt_time - last_interrupt_time_btnUp > DEBOUNCE_DELAY) {
//        interruptCnt_btnUp++;
//    }
//
//    last_interrupt_time_btnUp = this_interrupt_time;
//}
//
//void isr_btnUp() {
//    interruptCnt_btnUp++;
//}
//
//void isr_btnDown() {
//    interruptCnt_btnDown++;
//}
//
//void isr_btnOk() {
////    isr_btnUpDebounce();
//    interruptCnt_btnOk++;
//}

void storeSettings(HeaterSettings settings) {
    int eeAddress = EEPROM_SETTINGS;
    EEPROM.put(eeAddress, settings);
    Serial.println("Settings saved.");
}


#define DEFAULT_TARGET_TEMP 160

HeaterSettings loadSettings() {
    int eeAddress = EEPROM_SETTINGS;
    HeaterSettings settings{};
    EEPROM.get(eeAddress, settings);

    if (settings.targetTemp <= 0.0) {
        settings.targetTemp = DEFAULT_TARGET_TEMP;
    }
    return settings;
}

HeaterSettings currentSettings{};

AceButton btn_Up(PIN_BTN_UP);
AceButton btn_Down(PIN_BTN_DOWN);
AceButton btn_Ok(PIN_BTN_OK);

void handleEvent(AceButton*, uint8_t, uint8_t);

void setup() {
    delay(1000); // some microcontrollers reboot twice

    // Init Arduino features
    Serial.begin(115200);
    while (! Serial); // Wait until Serial is ready - Leonardo/Micro

    Serial.println(F("setup(): begin"));

    if (!i2CAddrTest(0x27)) {
        lcd = LiquidCrystal_I2C(0x3F, 16, 2);
    }

    analogReference(EXTERNAL); // Connect AREF to 3.3V and use that, less noisy!

    currentSettings = loadSettings();

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

    // Buttons
    // Configure the ButtonConfig with the event handler, and enable all higher
    // level events.
    ButtonConfig* buttonConfig = ButtonConfig::getSystemButtonConfig();
    buttonConfig->setEventHandler(handleEvent);
    buttonConfig->setFeature(ButtonConfig::kFeatureClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    buttonConfig->setFeature(ButtonConfig::kFeatureRepeatPress);

    // Interrupts
    // enableInterrupt(PIN_BTN_UP, isr_btnUpDebounce, FALLING);
    // enableInterrupt(PIN_BTN_DOWN, isr_btnDown, FALLING);
    // enableInterrupt(PIN_BTN_OK, isr_btnOk, FALLING);

    // Check if the button was pressed while booting
    if (btn_Up.isPressedRaw()) {
        Serial.println(F("setup(): btn_Up was pressed while booting"));
    }
    if (btn_Down.isPressedRaw()) {
        Serial.println(F("setup(): btn_Up was pressed while booting"));
    }
    if (btn_Ok.isPressedRaw()) {
        Serial.println(F("setup(): btn_Ok was pressed while booting"));
    }

    Serial.println(F("setup(): ready"));
}

bool led, heaterEnable = false;


float readTemp(uint8_t NUMSAMPLES = 5) {

    float average = 0;
    // take N samples in a row, with a slight delay
    for (int i = 0; NUMSAMPLES > i; i++) {
        average += analogRead(PIN_TEMP);
        delay(10);
    }

    // average samples
    average /= NUMSAMPLES;

    float AREF = 3.3;
    int Ro = 100, B =  3950; //Nominal resistance 50K, Beta constant
    float Rseries = 4.7;// Series resistor 10K
    float To = 298.15; // Nominal Temperature

    float Vi = average * (AREF / 1023.0);
    //Convert voltage measured to resistance value
    //All Resistance are in kilo ohms.
    float R = (Vi * Rseries) / (AREF - Vi);
    /*Use R value in steinhart and hart equation
      Calculate temperature value in kelvin*/
    float T =  1 / ((1 / To) + ((log(R / Ro)) / B));
    float Tc = T - 273.15; // Converting kelvin to celsius
    float Tf = Tc * 9.0 / AREF + 32.0; // Converting celsius to Fahrenheit

    return Tc;
}

#define MIN_TEMP 20
#define MAX_TEMP 220

// Thermistor lookup table for RepRap Temperature Sensor Boards (http://make.rrrf.org/ts)
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4700 --beta=3985 --max-adc=1023
// r0: 100000
// t0: 25
// r1: 0
// r2: 4700
// beta: 3985
// max adc: 1023
#define NUMTEMPS 20
float tempTable[NUMTEMPS][2] = {
        {1,    906},
        {54,   263},
        {107,  215},
        {160,  189},
        {213,  171},
        {266,  157},
        {319,  145},
        {372,  135},
        {425,  126},
        {478,  118},
        {531,  110},
        {584,  103},
        {637,  95},
        {690,  88},
        {743,  80},
        {796,  71},
        {849,  62},
        {902,  50},
        {955,  34},
        {1008, 2}
};

float targetTemp = 150;

void toggle_led();

void check_buttons();

void update_state(float temp, bool heaterEnabled);

void update_lcd(float temp, bool b);

//float read_temp(int samples);

void toggle_led();


bool toggle_heater();

void temp_decrease();

void temp_increase();

void loop() {

    float temp = readTemp(5);

    update_lcd(temp, heaterEnable);
    toggle_led();

    check_buttons();

    update_state(temp, heaterEnable);

    delay(250); // refresh rate
}

void update_state(float temp, bool heaterEnabled) {
    if (temp < targetTemp && heaterEnabled) {
        digitalWrite(PIN_HEATER, 1);
    } else {
        digitalWrite(PIN_HEATER, 0);
    }
}

void check_buttons() {
    btn_Up.check();
    btn_Down.check();
    btn_Ok.check();
}

// The event handler for both buttons.
void handleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {

    // Print out a message for all events, for both buttons.
    Serial.print(F("handleEvent(): pin: "));
    Serial.print(button->getPin());
    Serial.print(F("; eventType: "));
    Serial.print(AceButton::eventName(eventType));
    Serial.print(F("; buttonState: "));
    Serial.println(buttonState);

    // Control the LED only for the Pressed and Released events of Button 1.
    // Notice that if the MCU is rebooted while the button is pressed down, no
    // event is triggered and the LED remains off.
    switch (eventType) {
        case AceButton::kEventPressed:
            if (button->getPin() == PIN_BTN_OK) {
                bool okPressed = true;
            }
            break;
        case AceButton::kEventReleased:
            if (button->getPin() == PIN_BTN_OK) {
                toggle_heater();
            }
            break;
        case AceButton::kEventClicked:
            if (button->getPin() == PIN_BTN_UP) {
                temp_increase();
            } else if (button->getPin() == PIN_BTN_DOWN) {
                temp_decrease();
            }
            break;
    }
}

void temp_increase() {
    if (currentSettings.targetTemp < MAX_TEMP) {
        currentSettings.targetTemp++;
    }
}

void temp_decrease() {
    if (currentSettings.targetTemp > MIN_TEMP) {
        currentSettings.targetTemp--;
    }
}

// toggle heater state and return the new state
bool toggle_heater() {
    heaterEnable = !heaterEnable;
    return heaterEnable;
}

bool i2CAddrTest(uint8_t addr) {
    Wire.begin();
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
        return true;
    }
    return false;
}

void toggle_led() {
    led = !led;
    digitalWrite(LED_BUILTIN, led);
}

//@deprecated
//float read_temp(int samples = 1) {
//    int i;
//
//    float rawTemp = 0;
//    for (i = 0; i < samples; i++) {
//        rawTemp += analogRead(PIN_TEMP);
//    }
//    rawTemp /= samples;
//
//    float celsius = 0;
//
//    for (i = 1; i < NUMTEMPS; i++) {
//        if (tempTable[i][0] > rawTemp) {
//            float realTemp = tempTable[i - 1][1]
//                             + (rawTemp - tempTable[i - 1][0])
//                               * (tempTable[i][1] - tempTable[i - 1][1])
//                               / (tempTable[i][0] - tempTable[i - 1][0]);
//
//            if (realTemp > 255) {
//                realTemp = 255;
//            }
//
//            celsius = realTemp;
//
//            break;
//        }
//    }
//
//    // Overflow: We just clamp to 0 degrees celsius
//    if (i == NUMTEMPS)
//        celsius = 0;
//
//    return celsius;
//}

void update_lcd(float temp, bool heaterEnabled) {
    lcd.setCursor(0, 0);  // set the cursor to column 0, line 0
    lcd.print("T: ");
    lcd.print(temp);
    lcd.print("/");
    lcd.print(round(targetTemp));

    lcd.setCursor(0, 1);
    lcd.print("Heater: ");
    lcd.print(((heaterEnabled) ? "ARMED" : "Safe "));
}
