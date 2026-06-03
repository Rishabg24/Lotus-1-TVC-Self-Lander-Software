// TVC (Thrust Vector Control) — quaternion error → servo angle pipeline.
//
// Coordinate convention (body frame, right-hand rule):
//   +X  →  forward / "nose" direction (pitch nose-up is +θ)
//   +Y  →  right wing / lateral
//   +Z  →  up (thrust axis when rocket is vertical)
//
// Quaternion storage: q[0]=w, q[1]=x, q[2]=y, q[3]=z
//
// Error quaternion:  qErr = conj(qCurr) * qTarget
//   When qTarget = identity this simplifies to conj(qCurr), which encodes
//   "how much and in which direction must I rotate to get back to vertical."
//   Because target is always identity and conj(I * conj(q)) == conj(q),
//   world-frame vs body-frame multiplication order doesn't matter here.
//
// Euler extraction uses standard ZYX (aerospace) convention:
//   Pitch θ (about Y): asin( 2·(w·y − x·z) )
//   Yaw   ψ (about Z): atan2( 2·(w·z + x·y),  1 − 2·(y²+z²) )
//   Roll  φ is ignored — we don't control it.
//
// Servo sign convention:
//   servoX → controls the PITCH axis
//   servoY → controls the YAW   axis
//   Positive error  →  positive deflection (TVC nozzle tip moves in the
//   same direction as the tilt, generating restoring torque about CoM).
//   Verify sign empirically on the bench before flight.

#include "tvc.h"
#include <Arduino.h>
#include <cmath>
#include "pid.h"

TVC::TVC(float max_gimbal_angle)
    : max_deflection_angle_rad(max_gimbal_angle),
      ServoX(0.0f), ServoY(0.0f),
      pitch(), yaw()
{
    targetQuat[0] = 1.0f; // w
    targetQuat[1] = 0.0f; // x
    targetQuat[2] = 0.0f; // y
    targetQuat[3] = 0.0f; // z
}

void TVC::init(float kp, float ki, float kd, float maxAngle)
{
    pitch = PID(kp, ki, kd);
    yaw = PID(kp, ki, kd);
    pitch.reset();
    yaw.reset();

    max_deflection_angle_rad = maxAngle;

    targetQuat[0] = 1.0f;
    targetQuat[1] = 0.0f;
    targetQuat[2] = 0.0f;
    targetQuat[3] = 0.0f;

    ServoX = 0.0f;
    ServoY = 0.0f;
}

void TVC::update(float q0, float q1, float q2, float q3, float dt)
{
    float qCurr[4] = {q0, q1, q2, q3};
    float qErr[4];
    computeErrorQuat(qCurr, qErr);

    float pitchError = 0.0f;
    float yawError = 0.0f;
    extractControlErrors(qErr, &pitchError, &yawError);

    float pitchControl = pitch.compute(pitchError, dt,
                                       max_deflection_angle_rad,
                                       -max_deflection_angle_rad);
    float yawControl = yaw.compute(yawError, dt,
                                   max_deflection_angle_rad,
                                   -max_deflection_angle_rad);

    mapToServoAngles(pitchControl, yawControl, &ServoX, &ServoY);
}

void TVC::computeErrorQuat(float qCurr[4], float qErr[4])
{
    // qErr = conj(qCurr) * qTarget
    // conj(qCurr) = (w, -x, -y, -z)
    float w = qCurr[0];
    float x = -qCurr[1];
    float y = -qCurr[2];
    float z = -qCurr[3];

    // Quaternion multiply:  qErr = conj(qCurr) * targetQuat
    // (When target is identity this is just conj(qCurr), but written
    //  explicitly so the formula is correct for any target.)
    qErr[0] = w * targetQuat[0] - x * targetQuat[1] - y * targetQuat[2] - z * targetQuat[3];
    qErr[1] = w * targetQuat[1] + x * targetQuat[0] + y * targetQuat[3] - z * targetQuat[2];
    qErr[2] = w * targetQuat[2] - x * targetQuat[3] + y * targetQuat[0] + z * targetQuat[1];
    qErr[3] = w * targetQuat[3] + x * targetQuat[2] - y * targetQuat[1] + z * targetQuat[0];

    // Quaternion double-cover: always take the shorter-arc representative.
    if (qErr[0] < 0.0f)
    {
        qErr[0] = -qErr[0];
        qErr[1] = -qErr[1];
        qErr[2] = -qErr[2];
        qErr[3] = -qErr[3];
    }
}

void TVC::extractControlErrors(float qErr[4], float *pitchError, float *yawError)
{

    // Using a vertical, or traditional coordinate system:
    // X: parallel to ground and rotational axis for Pitch
    // Y: parallel to ground and roataional axis for Yaw
    // Z: perpendicular to ground and rotational axis for roll

    // Using axis-angle extraction to avoid gimabl lock. Essentially computing the projection of the total rotation vector
    // onto each axis

    float w = qErr[0], x = qErr[1], y = qErr[2], z = qErr[3];

    float vec_mag = sqrtf(x*x + y*y + z*z);

    if (vec_mag < 1e-6f){
        *pitchError = 0.0f;
        *yawError = 0.0f;
        return;
    }

    float theta = 2.0f * atan2f(vec_mag, w); // rotation angle

    float scaleFactor = theta / vec_mag; // scale vector part to get correct angle

    *pitchError = x * scaleFactor; // rotation about X axis
    *yawError = y * scaleFactor;   // rotation about Y axis
    
}

void TVC::mapToServoAngles(float pitchControl, float yawControl,
                           float *servoX_out, float *servoY_out)
{
    // ServoX → Yaw axis, ServoY → Pitch axis.
    //
    // Sign convention: positive error means the rocket has tilted in the
    // positive direction; we command the nozzle to deflect the same way,
    // which (because the thrust acts below the CoM) produces a restoring
    // torque. Verify direction with a bench spin test
    *servoX_out = constrain(pitchControl, -max_deflection_angle_rad, max_deflection_angle_rad);
    *servoY_out = constrain(yawControl, -max_deflection_angle_rad, max_deflection_angle_rad);
}

float TVC::getServoX() { return ServoX; }
float TVC::getServoY() { return ServoY; }

void TVC::disable()
{
    ServoX = 0.0f;
    ServoY = 0.0f;
    pitch.reset();
    yaw.reset();
}  