//
// Created by DJ on 7/04/2024.
//

#ifndef HOTPLATE_RESCUE_DISPLAY_H
#define HOTPLATE_RESCUE_DISPLAY_H

#include "stdint.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_ADDR 0x3C

class Display {
    explicit Display(uint8_t i2cAddr);

    Adafruit_SSD1306 *display;
};


#endif //HOTPLATE_RESCUE_DISPLAY_H
