//
// Created by DJ on 27/06/2024.
//

#if DEBUG
#include "app_api.h"
#include "avr8-stub.h"
#endif

#include "main.h"
#include "metric.h"
#include "pid.h"
#include "reflow_profile.h"

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <RunningAverage.h>
#include <TaskScheduler.h>
#include <Wire.h>
#include <rotary.h>

// Next:
// - do task vars need to be volatile?
// - show time remaining/elapsed on display
// - led status indicator?
// Last:
// - watchdog check for thermal runaway

volatile unsigned long taskElapsed;

void soakTaskHandler();
void reflowTaskHandler();
void cooldownTaskHandler();

Scheduler scheduler;
Task soakTask(TASK_SECOND, TASK_FOREVER, soakTaskHandler);
Task reflowTask(TASK_SECOND, TASK_FOREVER, reflowTaskHandler);
Task cooldownTask(TASK_SECOND, TASK_FOREVER, cooldownTaskHandler);

LiquidCrystal_I2C lcd(I2C_ADDR, 16, 2);

// Global variables
volatile float setTemp = 0;

volatile unsigned int encoderValue;

Rotary encoder(ENCODER_PIN_CLK, ENCODER_PIN_DT);
RunningAverage temp(TEMP_SAMPLES);
State currentState = IDLE;

State advanceState(const State _state) {
  switch (_state) {
    case IDLE:
      return SOAK;

    case SOAK:
      return REFLOW;

    case REFLOW:
      return COOLDOWN;

    case COOLDOWN:
    default:
      return IDLE;
  }
}

/**
 * Handler for pin change interrupts for D0 to D7
 */
ISR(PCINT2_vect) {
  // Holds the pin states from the last interrupt
  static uint8_t prevPINDState = 0b00000000;
  static unsigned long last_exec = 0;

  if ((millis() - last_exec) < 1) { // debounce
    return;
  }

  last_exec = millis();

  const uint8_t changedBits = PIND ^ prevPINDState; // XOR to find changed bits
  prevPINDState = PIND; // Update for next time

  // D3 and/or D4 (PCINT19 and PCINT20) have changed
  if (changedBits & (bit(PCINT19) | bit(PCINT20))) {
    checkEncoder();
  }

} // end of PCINT2_vect

/**
 * Configures the PWM timer for the given pin to Fast 8-bit PWM (Mode 5 for Timer 1, Mode 3 for Timer 0 and Timer 2)
 * and configures the clock-select to no prescaler by setting CSn2:0 to "001".
 */
bool configurePwmPrescaler(uint8_t pin) {
  switch (digitalPinToTimer(pin)) {
    case TIMER0A:
    case TIMER0B:
      // Set Fast PWM mode, 8-bit for Timer 0 (Mode 3)
      TCCR0A = (TCCR0A & 0b11111100) | bit(WGM00) | bit(WGM01); // WGM01:0 = 11
      TCCR0B = (TCCR0B & 0b11111000) | bit(CS00); // CS02:0 = 001
      break;
    case TIMER1A:
    case TIMER1B:
      // Set Fast PWM mode, 8-bit for Timer 1 (Mode 5)
      TCCR1A = (TCCR1A & 0b11111100) | bit(WGM10); // WGM11:10 = 01
      TCCR1B = (TCCR1B & 0b11101000) | bit(WGM12) | bit(CS10); // WGM13:12 = 01, CS12:0 = 001
      break;
    case TIMER2A:
    case TIMER2B:
      // Set Fast PWM mode, 8-bit for Timer 2 (Mode 3)
      TCCR2A = (TCCR2A & 0b11111100) | bit(WGM20) | bit(WGM21); // WGM21:0 = 11
      TCCR2B = (TCCR2B & 0b11111000) | bit(CS20); // CS22:0 = 001
      break;
    case NOT_ON_TIMER:
    default:
      // If the pin is not on a timer, return false
      return false;
  }

  return true;
} // end of configurePwmPrescaler

void startLcd() {
  lcd.begin(16, 2);
  lcd.clear();
  lcd.backlight();
}

void setup() {
#if DEBUG
  debug_init();
#elif SERIAL_TELEMETRY
  Serial.begin(115200); // Initialize serial communication
#endif

  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(THERMISTOR_PIN, INPUT);
  // handled by Rotary.cpp already
  pinMode(ENCODER_PIN_CLK, INPUT);
  pinMode(ENCODER_PIN_DT, INPUT);

  Wire.begin(); // Initialize I2C communication
  delay(100); // Wait for things to settle
  startLcd();

  if (!configurePwmPrescaler(HEATER_PIN)) {
    // Error handling if the pin does not support PWM

    lcd.setCursor(0, 0);
    lcd.print("*BAD HEATER PIN*");
    lcd.setCursor(0, 1);
    char errDetail[32] = {};
    int errLen = sprintf(errDetail, "Pin %d has no PWM support", HEATER_PIN);
    lcd.print(errDetail);
    while (true) {
      delay(300);
      lcd.scrollDisplayLeft();
    }
  }

  encoderValue = static_cast<int>(setTemp);

  enablePinChangeInterrupt(ENCODER_PIN_CLK);
  enablePinChangeInterrupt(ENCODER_PIN_DT);

  printDisplayFurniture();

  temp.fillValue(readTemperature(), TEMP_SAMPLES);
  scheduler.addTask(soakTask);
  scheduler.addTask(reflowTask);
  scheduler.addTask(cooldownTask);
  scheduler.disableAll();
  scheduler.enable();
}

void loop() {
  // encoderValue = constrain(encoderValue, 0, 300);
  // if (encoderValue != setTemp) {
  //   setTemp = encoderValue;
  // }

  if (getButtonState() == LONG_PRESS) {
    currentState = advanceState(currentState);
  }

  // Set the active task based on the current state
  switch (currentState) {

    case SOAK:
      if (reflowTask.isEnabled()) reflowTask.disable();
      if (cooldownTask.isEnabled()) cooldownTask.disable();
      if (!soakTask.enableIfNot()) {
        setTemp = min(round(temp.getFastAverage()), PROFILE_SOAK_TEMP);
      }
      break;

    case REFLOW:
      if (soakTask.isEnabled()) soakTask.disable();
      if (cooldownTask.isEnabled()) cooldownTask.disable();
      if (!reflowTask.enableIfNot()) {
        setTemp = min(round(temp.getFastAverage()), PROFILE_REFLOW_TEMP);
      }
      break;

    case COOLDOWN:
      if (soakTask.isEnabled()) soakTask.disable();
      if (reflowTask.isEnabled()) reflowTask.disable();
      if (!cooldownTask.enableIfNot()) {
        setTemp = max(round(temp.getFastAverage()), PROFILE_COOLDOWN_TEMP);
      }

    case IDLE:
    default:
      break;
  }

  // Run the scheduler task(s)
  scheduler.execute();

  // Update the heater output
  if (currentState != IDLE) {
    analogWrite(HEATER_PIN, calculatePID(setTemp, temp.getFastAverage()));
  } else {
    scheduler.disableAll();
    digitalWrite(HEATER_PIN, LOW);
  }

  // Update the temperature buffer
  temp.addValue(readTemperature());

  // Update the LCD display
  updateDisplay();

  // Telemetry
  _PM(Metric("target", setTemp));
  _PM(Metric("temp", temp.getFastAverage()));

  // delay(10);
}

/**
 * TEMP
 */

float readTemperature() {
  const int adcValue = analogRead(THERMISTOR_PIN); // Read the analog value
  const float resistance = SERIES_RESISTOR / (1023.0 / adcValue - 1); // Calculate resistance

  // Calculate temperature using the Steinhart-Hart equation
  float steinhart = resistance / THERMISTOR_NOMINAL; // (R/Ro)
  steinhart = log(steinhart); // ln(R/Ro)   NOLINT(*-narrowing-conversions, *-type-promotion-in-math-fn)
  steinhart /= B_COEFFICIENT; // 1/B * ln(R/Ro)
  steinhart += 1.0f / (TEMPERATURE_NOMINAL + 273.15f); // + (1/To)
  steinhart = 1.0f / steinhart; // Invert
  steinhart -= 273.15f; // Convert to Celsius

  return steinhart;
}

/**
 * LCD DISPLAY
 */

/**
 * Starts the display and prints the static portions
 */
void printDisplayFurniture() {
  startLcd();

  lcd.setCursor(0, 0);
  lcd.print("Set:");
  lcd.setCursor(0, 1);
  lcd.print("Cur:");
} // end of initDisplay

/**
 * Update the display based on the current state
 */
void updateDisplay() {
  static float lastSetTemp, lastTemp;
  const float currentTemp = temp.getFastAverage();

  // Set the cursor position to print the set temperature
  lcd.setCursor(5, 0);
  if (currentState == SOAK || currentState == REFLOW || currentState == COOLDOWN) {
    if (lastSetTemp != setTemp) {
      constexpr size_t tempBuffSize = 8;
      char tempBuff[tempBuffSize] = {};
      lastSetTemp = setTemp;
      dtostrf(lastSetTemp, 5, 1, tempBuff); // convert to xxx.yy string

      lcd.print(tempBuff);
      lcd.print("C");
    }
  } else {
    lcd.print("---.-C");
  }

  if (lastTemp != currentTemp) {
    lastTemp = currentTemp;
    constexpr size_t tempBuffSize = 8;
    char tempBuff[tempBuffSize] = {};
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

  lcd.setCursor(12, 1);
  if (currentState == SOAK || currentState == REFLOW) {
    if (taskElapsed > 0) {
      uint8_t mins = taskElapsed / 60;
      uint8_t secs = taskElapsed % 60;
      lcd.print(mins);
      lcd.print(":");
      if (secs < 10) {
        lcd.print("0");
      }
      lcd.print(secs);
    } else {
      lcd.print("   ^");
    }
  } else if (currentState == COOLDOWN) {
    lcd.print("   v");
  } else {
    lcd.print("    ");
  }
} // end of updateDisplay

/**
 * Updates the Encoder state machine and checks for rotation events
 */
void checkEncoder() {
  const uint8_t encoderState = encoder.process();
  if (encoderState == DIR_CW && encoderValue < TEMP_MAX) {
    encoderValue++;
  } else if (encoderState == DIR_CCW && encoderValue > TEMP_MIN) {
    encoderValue--;
  }
} // end of checkEncoder

void enablePinChangeInterrupt(const uint8_t pin) {
  // uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmsk = digitalPinToPCMSK(pin);
  const uint8_t PCICRbit = digitalPinToPCICRbit(pin);
  const uint8_t PCMSKbit = digitalPinToPCMSKbit(pin);

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

  const unsigned long now = millis();
  ButtonResult result = RELEASED;

  if (digitalRead(BUTTON_PIN)) { // button held
    if (buttonPressStart == 0) { // is this the start of the press?
      buttonPressStart = now;
    }

    timeHeld = now - buttonPressStart; // Update time held

    if (timeHeld >= BUTTON_LONG_PRESS_MILLIS &&
      newPress) { // long-press acheivement unlocked!
      result = LONG_PRESS;
      newPress = false;
    } else if (timeHeld >=
      BUTTON_SHORT_PRESS_MILLIS) { // basically a debounce...
      result = HELD;
    }

  } else { // button released
    result = (timeHeld >= BUTTON_SHORT_PRESS_MILLIS && newPress)
        ? SHORT_PRESS
        : RELEASED;

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

/**
 * REFLOW PROFILE TASKS
 */
void soakTaskHandler() {
  if (soakTask.isFirstIteration()) {
    taskElapsed = 0;
  }
  if (setTemp < PROFILE_SOAK_TEMP) { // ramp-up to soak temp
    setTemp += PROFILE_SOAK_RAMP_RATE;
  } else { // hold soak temp for PROFILE_SOAK_DURATION
    if (++taskElapsed >= PROFILE_SOAK_DURATION) {
      currentState = advanceState(currentState);
    }
  }
}

void reflowTaskHandler() {
  if (reflowTask.isFirstIteration()) {
    taskElapsed = 0;
  }

  if (setTemp < PROFILE_REFLOW_TEMP) { // ramp-up to soak temp
    setTemp = min((setTemp + PROFILE_REFLOW_RAMP_RATE), PROFILE_REFLOW_TEMP);
  } else { // hold soak temp for PROFILE_SOAK_DURATION
    if (++taskElapsed >= PROFILE_REFLOW_DURATION) {
      currentState = advanceState(currentState);
    }
  }
}
void cooldownTaskHandler() {
  if (cooldownTask.isFirstIteration()) {
    taskElapsed = 0;
  }
  if (setTemp > PROFILE_COOLDOWN_TEMP) { // ramp-up to soak temp
    setTemp = max(setTemp - PROFILE_COOLDOWN_RAMP_RATE, PROFILE_COOLDOWN_TEMP);
  } else if (temp.getFastAverage() <= PROFILE_COOLDOWN_TEMP) { // hold soak temp for PROFILE_SOAK_DURATION
    currentState = advanceState(currentState);
  }
  ++taskElapsed;
}
