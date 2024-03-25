//
// Created by DJ on 1/12/2023.

#include "main.h"
//#include "adc.h"
#include <EEPROM.h>
#include "EnableInterrupt.h"
#include "settings.h"


#define PIN_TEMP PIN_A0
#define PIN_ISENSE PIN_A1
#define PIN_HEATER 3
// #define PIN_BTN_UP 10
#define PIN_BTN_OK 7
// #define PIN_BTN_DOWN 8

#define MIN_TEMP 20
#define MAX_TEMP 220
#define SAMPLE_DELAY 2

#define DEFAULT_TARGET_TEMP 160
#define EEPROM_SETTINGS_ADDR 0

#define CLK_PIN 8
#define DT_PIN 10
#define SW_PIN 7

#define DIRECTION_CW 0   // clockwise direction
#define DIRECTION_CCW 1  // counter-clockwise direction
#define DEBOUNCE 10
volatile int counter = DEFAULT_TARGET_TEMP;
volatile int direction = DIRECTION_CW;
volatile unsigned long last_time;  // for debouncing
int prev_counter;

#define REFRESH_RATE 500
#define BAUD 115200

HeaterSettings settings(EEPROM_SETTINGS_ADDR);


bool led, heaterEnable = false;
int saveSettings = 0;

float displayTemp = 0;

bool i2CAddrTest(uint8_t addr);

bool check_refresh();

void check_save();

bool loadSettings();

void ISR_encoderChange() {
  if ((millis() - last_time) < DEBOUNCE) 
    return;

  if (digitalRead(DT_PIN) == HIGH) {
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    temp_increase();
    direction = DIRECTION_CW;
  } else {
    // the encoder is rotating in clockwise direction => increase the counter
    temp_decrease();
    direction = DIRECTION_CCW;
  }

  last_time = millis();
}

// 1. Configure pins
// 2. Configure peripherals
void setup() {

    // Init Arduino features
    Serial.begin(BAUD);
    while (! Serial); // Wait until Serial is ready

    Serial.print(F("setup(): serial started @ baud rate ")); Serial.println(BAUD)

    if (!i2CAddrTest(0x27)) {
        lcd = LiquidCrystal_I2C(0x3F, 16, 2);
    }

    analogReference(EXTERNAL); // Connect AREF to 3.3V and use that, less noisy!

    // Init LCD
    lcd.init();
    lcd.backlight();

    // Init inputs
    pinMode(PIN_TEMP, INPUT);
    pinMode(PIN_ISENSE, INPUT);
    // pinMode(PIN_BTN_UP, INPUT);
    pinMode(PIN_BTN_OK, INPUT);
    // pinMode(PIN_BTN_DOWN, INPUT);

    
    // configure encoder pins as inputs
    pinMode(CLK_PIN, INPUT);
    pinMode(DT_PIN, INPUT);

    // use interrupt for CLK pin is enough
    // call ISR_encoderChange() when CLK pin changes from LOW to HIGH
    // attachInterrupt(digitalPinToInterrupt(CLK_PIN), ISR_encoderChange, RISING);
    enableInterrupt(CLK_PIN, ISR_encoderChange, RISING);

    // Init outputs
    pinMode(PIN_HEATER, OUTPUT);

    // Buttons
    // Configure the ButtonConfig with the event handler, and enable all higher
    // level events.
    // ButtonConfig* buttonConfig = ButtonConfig::getSystemButtonConfig();
    // buttonConfig->setEventHandler(handleEvent);
    // buttonConfig->setFeature(ButtonConfig::kFeatureClick);
    // buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
    // buttonConfig->setFeature(ButtonConfig::kFeatureRepeatPress);

    // Check if the button was pressed while booting
    // if (btn_Up.isPressedRaw()) {
    //     Serial.println(F("setup(): btn_Up was pressed while booting"));
    // }
    // if (btn_Down.isPressedRaw()) {
    //     Serial.println(F("setup(): btn_Up was pressed while booting"));
    // }
    // if (btn_Ok.isPressedRaw()) {
    //     Serial.println(F("setup(): btn_Ok was pressed while booting"));
    // }

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
    // float Tf = Tc * 9.0 / AREF + 32.0; // Converting celsius to Fahrenheit

    return Tc;
}


bool refresh;
volatile bool forceRefresh;

typedef enum state_t {
    S_INITIALISE, // 0
    S_IDLE,  // 1
    S_MENU, // 2
    S_PREHEAT, // 3
    S_REFLOW, // 4
    S_COOLDOWN, // 5
    S_HALT  // 6
};

void loop() {
    static state_t state = S_IDLE; // initial state is 1, the "idle" state.
    switch (state) {
        case S_INITIALISE:
            if (loadSettings()) {
                state = S_IDLE;
            }
            break;
        case S_IDLE:
            // We don't need to do anything here, waiting for a forced state change.
            break;
        default:
            state = S_IDLE;
            break;
    }


    float temp = readTemp(3);

    if (!forceRefresh) {
        displayTemp = temp;
    }

    if (check_refresh()) {
        update_lcd(displayTemp);
        toggle_led();
    }

    update_heater_state(temp);

    check_save();
}

bool loadSettings() {
    return true;
}


// check if it's time to refresh the display
bool check_refresh() {
    static unsigned long lastRefresh = 0;

    if (!forceRefresh && ((millis() - lastRefresh) < REFRESH_RATE)) {
        return false;
    }

    lastRefresh = millis();
    forceRefresh = false;
    return true;
}

// toggle the heater state based on the current temperature
void update_heater_state(float temp) {
    if (temp < currentSettings.targetTemp && heaterEnable) {
        digitalWrite(PIN_HEATER, 1);
    } else {
        digitalWrite(PIN_HEATER, 0);
    }
}

void check_buttons() {
    // btn_Ok.check();
}



void temp_increase() {
    if (currentSettings.targetTemp < MAX_TEMP) {
        currentSettings.targetTemp++;
    }
    forceRefresh = true;
    saveSettings = 1;
}

void temp_decrease() {
    if (currentSettings.targetTemp > MIN_TEMP) {
        currentSettings.targetTemp--;
    }
    forceRefresh = true;
    saveSettings = 1;
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
    if (temp < 100) { lcd.print(" "); }
    lcd.print(temp);
    lcd.print("/");
    lcd.print(currentSettings.targetTemp);
    lcd.print("   ");

    lcd.setCursor(0, 1);
    lcd.print("Heater: ");
    lcd.print(((heaterEnable) ? "ARMED" : "Safe "));

    Serial.print("Temp: "); Serial.println(temp); 
    Serial.print("Target: "); Serial.println(currentSettings.targetTemp);
}
