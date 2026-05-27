#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);

    // Compute PID output. UMAX/UMIN are in the same units as the output
    // (radians for gimbal angle). dt is in seconds.
    float compute(float error, float dt, float UMAX, float UMIN);

    // Zero the integral and previous-error state. Call before re-arming
    // or after a mode transition to prevent integral wind-up carry-over.
    void reset();

private:
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
};

#endif // PID_H