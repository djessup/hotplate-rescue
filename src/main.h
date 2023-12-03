#ifndef HOTPLATE_RESCUE_MAIN_H
#define HOTPLATE_RESCUE_MAIN_H

#include "Arduino.h"
#include "LiquidCrystal_I2C.h"

#define DISPLAY_ROWS 16
#define DISPLAY_COLS 2
#define DISPLAY_ADDR 0x27

LiquidCrystal_I2C lcd(DISPLAY_ADDR, DISPLAY_ROWS, DISPLAY_COLS);  // set the LCD address and the number of columns and rows

#endif //HOTPLATE_RESCUE_MAIN_H
