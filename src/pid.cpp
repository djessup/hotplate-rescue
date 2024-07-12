#include "pid.h"
#include "metric.h"

/**
 * PID CONTROLLER
 *
 * Calculates the PWM duty for the heater using a PID control loop
 */
uint8_t calculatePID(float setpoint, float input) {
    static float integralTerm = 0.0f;
    static float lastInput = 0.0f;
    static float derivativeFilter = 0.0f; // For filtering the derivative term
    static const float alpha = 0.01f; // Smoothing factor for derivative filtering
    static unsigned long lastTime = 0;

    // Update the current time
    unsigned long currentTime = millis();
    unsigned long timeChange = (currentTime - lastTime);
    float timeChangeSec = (timeChange / 1000.0f);

    // Compute the error
    float error = setpoint - input;

    // Proportional term
    float Pout = Kp * error;

    // Integral term
    integralTerm += (Ki * error * timeChangeSec);

    // Anti-windup: prevent integral term from getting too large
    // if (integralTerm > 255.0f) integralTerm = 255.0f;
    // else if (integralTerm < 0.0f) integralTerm = 0.0f;
    // Integral clamping
    if (integralTerm > 100.0f) integralTerm = 100.0f;
    else if (integralTerm < -100.0f) integralTerm = -100.0f;

    // Derivative term
    float derivative = (input - lastInput) / timeChangeSec;
    // float Dout = Kd * derivative;

    // Filter the derivative to dampen rapid fluctuations that can exacerbate oscillations
    derivativeFilter = (alpha * derivative) + ((1 - alpha) * derivativeFilter);
    float Dout = Kd * derivativeFilter;

    // Compute the total output
    float output = Pout + integralTerm - Dout;

    // Restrict the output to the range 0-PWM_MAX
    if (output > PWM_MAX) output = PWM_MAX;
    else if (output < 0.0f) output = 0.0f;

    // Remember the last input and time for the next calculation
    lastInput = input;
    lastTime = currentTime;

    // Telemetry
    _PM(Metric("pid.currentTime", currentTime));
    _PM(Metric("pid.timeChange", timeChange));
    _PM(Metric("pid.error", error));
    _PM(Metric("pid.derivative", derivative));
    _PM(Metric("pid.P", Pout));
    _PM(Metric("pid.I", integralTerm));
    _PM(Metric("pid.D", Dout));
    _PM(Metric("pid.output", output));

    return static_cast<uint8_t>(round(output));
}
