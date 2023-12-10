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
#define PIN_BTN_UP 10
#define PIN_BTN_OK 7
#define PIN_BTN_DOWN 8

#define MIN_TEMP 20
#define MAX_TEMP 220
#define SAMPLE_DELAY 2

#define DEFAULT_TARGET_TEMP 160
#define EEPROM_SETTINGS_ADDR 0

uint16_t lastRefresh = 0, refreshRate = 500;

struct HeaterSettings {
    float targetTemp;
};

HeaterSettings currentSettings{
        .targetTemp = DEFAULT_TARGET_TEMP,
};

AceButton btn_Up(PIN_BTN_UP);
AceButton btn_Down(PIN_BTN_DOWN);
AceButton btn_Ok(PIN_BTN_OK);

bool led, heaterEnable, saveSettings = false;

float lastTemp = 0;

bool i2CAddrTest(uint8_t addr);

bool check_refresh();

void storeSettings() {
    int eeAddress = EEPROM_SETTINGS_ADDR;
    EEPROM.put(eeAddress, currentSettings);
    saveSettings = false;
    Serial.println("Settings saved.");
}

HeaterSettings loadSettings() {
    int eeAddress = EEPROM_SETTINGS_ADDR;
    HeaterSettings settings{};
    EEPROM.get(eeAddress, settings);

    if (settings.targetTemp <= 0.0) {
        settings.targetTemp = DEFAULT_TARGET_TEMP;
    }
    return settings;
}


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
    buttonConfig->setFeature(ButtonConfig::kFeatureRepeatPress);

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


float readTemp(uint8_t NUMSAMPLES = 5) {

    float average = 0;
    // take N samples in a row, with a slight delay
    for (int i = 0; NUMSAMPLES > i; i++) {
        average += analogRead(PIN_TEMP);
        delay(SAMPLE_DELAY);
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



void loop() {
    bool refresh = check_refresh();

    check_buttons();

    float temp = readTemp(5);
    lastTemp = temp;

    check_buttons(); // again, more responsive \__("o")__/

    if (refresh) {
        update_lcd(temp);
        toggle_led();
    }

//    Serial.print("millis: "); Serial.println(millis());

    update_state(temp);

    if (saveSettings) {
        storeSettings();
    }
}

bool check_refresh() {
    long t = millis() / refreshRate;
    if (t > lastRefresh) {
        lastRefresh = t;
        return true;
    }
    return false;
}

void update_state(float temp) {
    if (temp < currentSettings.targetTemp && heaterEnable) {
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
        case AceButton::kEventRepeatPressed:
            if (button->getPin() == PIN_BTN_UP) {
                temp_increase();
            } else if (button->getPin() == PIN_BTN_DOWN) {
                temp_decrease();
            }
            update_lcd(lastTemp);
            break;
    }
}


void temp_increase() {
    if (currentSettings.targetTemp < MAX_TEMP) {
        currentSettings.targetTemp++;
    }
    saveSettings = true;
}

void temp_decrease() {
    if (currentSettings.targetTemp > MIN_TEMP) {
        currentSettings.targetTemp--;
    }
    saveSettings = true;
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


void update_lcd(float temp) {
    lcd.setCursor(0, 0);  // set the cursor to column 0, line 0
    lcd.print("T: ");
    lcd.print(temp);
    lcd.print("/");
    lcd.print(round(currentSettings.targetTemp));

    lcd.setCursor(0, 1);
    lcd.print("Heater: ");
    lcd.print(((heaterEnable) ? "ARMED" : "Safe "));
}
