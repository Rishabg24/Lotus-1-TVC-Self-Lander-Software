#ifndef TVC_H
#define TVC_H

#include <cmath>
#include "pid.h"

class TVC {
public:
    TVC(float max_gimbal_angle);

    // Call once at startup (or after a mode change) to set PID gains and
    // reset internal state.
    void init(float kp, float ki, float kd, float maxAngle);

    // Call every control-loop tick.  q0-q3 are the Madgwick quaternion
    // (w, x, y, z). dt is seconds since last call.
    void update(float q0, float q1, float q2, float q3, float dt);

    // ---- Sub-steps (public so they can be unit-tested individually) ----

    // qErr = conj(qCurr) * targetQuat  (body-frame error quaternion)
    void computeErrorQuat(float qCurr[4], float qErr[4]);

    // ZYX Euler extraction: pitch (about Y) and yaw (about Z).
    // Roll is ignored — TVC cannot control it.
    void extractControlErrors(float qErr[4], float* pitchError, float* yawError);

    // Clamp PID outputs to gimbal limits and assign to servo channels.
    void mapToServoAngles(float pitchControl, float yawControl,
                          float* servoX, float* servoY);

    // Current servo commands in radians.
    float getServoX();
    float getServoY();

    // Emergency stop: zero servos and reset PID state.
    void disable();

private:
    float max_deflection_angle_rad;
    float targetQuat[4];   // (1,0,0,0) = vertical
    float ServoX;          // radians; controls pitch axis
    float ServoY;          // radians; controls yaw axis
    PID   pitch;
    PID   yaw;
};

#endif // TVC_H