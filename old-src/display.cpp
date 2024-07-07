//
// Created by DJ on 7/04/2024.
//

#include "display.h"

Display::Display(uint8_t i2cAddr) {
    display = new Adafruit_SSD1306(128, 64, &Wire, -1);
    display->begin(SSD1306_EXTERNALVCC, i2cAddr);
    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    display->setCursor(0, 0);
    display->display();
}
