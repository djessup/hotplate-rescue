/**
 * Defines the parameters of the reflow profile to follow
 */
#ifndef HOTPLATE_RESCUE_REFLOW_PROFILE_H
#define HOTPLATE_RESCUE_REFLOW_PROFILE_H

// Target temps in celsius
#define PROFILE_SOAK_TEMP (125.00f)
#define PROFILE_REFLOW_TEMP (180.00f)
#define PROFILE_COOLDOWN_TEMP (50.00f)

// Time temp held once target is reached (in seconds)
#define PROFILE_SOAK_DURATION (120) // 2 min
#define PROFILE_REFLOW_DURATION (60) // 1 min

// Degrees per sec to ramp to the target temperature
#define PROFILE_SOAK_RAMP_RATE (1.0f)
#define PROFILE_REFLOW_RAMP_RATE (1.0f)
#define PROFILE_COOLDOWN_RAMP_RATE (2.0f)

#endif // HOTPLATE_RESCUE_REFLOW_PROFILE_H
