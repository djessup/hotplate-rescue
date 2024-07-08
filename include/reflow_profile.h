// Target temps in celsius
#define PROFILE_SOAK_TEMP 125
#define PROFILE_REFLOW_TEMP 180

// Time temp held once target is reached
#define PROFILE_SOAK_DURATION 120 // 2 min
#define PROFILE_REFLOW_DURATION 60 // 1 min

// Degrees/sec to change the temperature 
#define PROFILE_SOAK_RAMP_RATE 1
#define PROFILE_REFLOW_RAMP_RATE 1
#define PROFILE_COOLDOWN_RAMP_RATE 2

/**
 * Heat run 1
 * Start: 21.5c
 * Target: 125c
 * Time ~60s
 * 100/60
 */