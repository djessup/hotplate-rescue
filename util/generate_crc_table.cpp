//
// Created by DJ on 30/03/2024.
//

#include <iostream>

#include <stdint.h>

#define POLYNOMIAL 0x1021
uint16_t crcTable[256];


void generateCRCTable() {
    for (int i = 0; i < 256; ++i) {
        uint16_t crc = 0;
        uint16_t c = i << 8;
        for (int j = 0; j < 8; ++j) {
            if ((crc ^ c) & 0x8000) crc = (crc << 1) ^ POLYNOMIAL;
            else crc = crc << 1;
            c = c << 1;
        }
        crcTable[i] = crc;
    }
}


int main() {
    generateCRCTable();
    std::cout << "uint16_t crcTable[256] = {";
    for (int i = 0; i < 256; ++i) {
        std::cout << crcTable[i];
        if (i != 255) {
            std::cout << ", ";
        }
    }
    std::cout << "};" << std::endl;
    return 0;
}

