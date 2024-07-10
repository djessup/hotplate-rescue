/**
 * Defines the parameters of the reflow profile to follow
 */
#ifndef HOTPLATE_RESCUE_REFLOW_PROFILE_H
#define HOTPLATE_RESCUE_REFLOW_PROFILE_H

// Target temps in celsius
#define PROFILE_SOAK_TEMP 125
#define PROFILE_REFLOW_TEMP 180
#define PROFILE_COOLDOWN_TEMP 50

// Time temp held once target is reached
#define PROFILE_SOAK_DURATION 120 // 2 min
#define PROFILE_REFLOW_DURATION 60 // 1 min

// Degrees/sec to change the temperature 
#define PROFILE_SOAK_RAMP_RATE 1
#define PROFILE_REFLOW_RAMP_RATE 1
#define PROFILE_COOLDOWN_RAMP_RATE 2

#endif // HOTPLATE_RESCUE_REFLOW_PROFILE_H
