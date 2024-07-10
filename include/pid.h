#ifndef HOTPLATE_RESCUE_PID_H
#define HOTPLATE_RESCUE_PID_H

#include <Arduino.h>

// PID constants
#define Kp (80.00f)
#define Ki (3.00f)
#define Kd (150.00f)

uint8_t calculatePID(float setpoint, float input);

#endif // HOTPLATE_RESCUE_PID_H