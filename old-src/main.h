#ifndef HOTPLATE_RESCUE_MAIN_H
#define HOTPLATE_RESCUE_MAIN_H

#include "Arduino.h"
#include "LiquidCrystal_I2C.h"
#include "AceButton.h"

using namespace ace_button;

#define DISPLAY_ROWS 16
#define DISPLAY_COLS 2
#define DISPLAY_ADDR 0x27



void handleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState);

void check_buttons();

void update_heater_state(float temp);

void update_lcd(float temp);

void toggle_led();

bool toggle_heater();

void temp_decrease();

void temp_increase();

LiquidCrystal_I2C lcd(DISPLAY_ADDR, DISPLAY_ROWS, DISPLAY_COLS);  // set the LCD address and the number of columns and rows

#endif //HOTPLATE_RESCUE_MAIN_H
