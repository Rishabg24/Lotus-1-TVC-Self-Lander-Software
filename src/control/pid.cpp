// Quaternion-based PID controller for TVC gimbal angle control.
#include "pid.h"
#include <Arduino.h>
#include <cmath>

PID::PID(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), previous_error(0.0f), integral(0.0f)
{}

float PID::compute(float error, float dt, float UMAX, float UMIN) {
    const float epsilon = 1e-4f;

    // --- Derivative ---
    float derivative = (error - previous_error) / (dt + epsilon);
    previous_error = error;

    // --- Tentative output using current (pre-update) integral ---
    // We check saturation BEFORE updating the integral so that anti-windup
    // can decide whether to allow integration this step.
    float output = kp * error + ki * integral + kd * derivative;

    // --- Clamping anti-windup ---
    // Only integrate when:
    //   (a) output is not saturated, OR
    //   (b) output is saturated but the new error would drive it back (i.e.,
    //       integrating in this direction actually helps, not hurts).
    bool winding_up = (output > UMAX && error > 0.0f) ||
                      (output < UMIN && error < 0.0f);

    if (!winding_up) {
        integral += error * dt;

        // Hard-clamp the integral term itself so it can never alone exceed
        // the output range. This bounds recovery time after large disturbances.
        float integral_limit = (UMAX - UMIN) / (fabsf(ki) + epsilon);
        if      (integral >  integral_limit) integral =  integral_limit;
        else if (integral < -integral_limit) integral = -integral_limit;
    }

    // --- Final output with updated integral ---
    output = kp * error + ki * integral + kd * derivative;

    // --- Output saturation ---
    if      (output > UMAX) output = UMAX;
    else if (output < UMIN) output = UMIN;

    return output;
}

void PID::reset() {
    integral       = 0.0f;
    previous_error = 0.0f;
}