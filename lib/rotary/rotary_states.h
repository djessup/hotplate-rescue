//
// Created by DJ on 3/07/2024.
//

#ifndef HOTPLATE_RESCUE_ROTARY_STATES_H
#define HOTPLATE_RESCUE_ROTARY_STATES_H

#include "rotary.h"

#define R_START 0b00000000 // 00

#ifdef HALF_STEP
    // Use the half-step state table (emits a code at 00 and 11)
    #define R_CCW_BEGIN 0b00000001 // 01
    #define R_CW_BEGIN 0b00000010 // 10
    #define R_START_M 0b00000011
    #define R_CW_BEGIN_M 0b00000100
    #define R_CCW_BEGIN_M 0b00000101
#else
    // Use the full-step state table (emits a code at 00 only)
    #define R_CW_FINAL 0b00000001
    #define R_CW_BEGIN 0b00000010
    #define R_CW_NEXT 0b00000011
    #define R_CCW_BEGIN 0b00000100
    #define R_CCW_FINAL 0b00000101
    #define R_CCW_NEXT 0b00000110
#endif

#endif //HOTPLATE_RESCUE_ROTARY_STATES_H
