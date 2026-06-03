#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include "sensors/imu.h"
#include "sensors/baro.h"
#include "control/pid.h"
#include "control/tvc.h"
#include "flash/flightLogger.h"
#include "state_machine/state.h"

// Pull in test declarations only when a TEST_ flag is active.
// In flight builds this entire block — and tests.cpp — compiles out.
#if defined(TEST_I2C_SCAN) || defined(TEST_SPI_SCAN) ||    \
    defined(TEST_IMU_ACCEL) || defined(TEST_IMU_GYRO) ||   \
    defined(TEST_IMU_QUATERNION) || defined(TEST_SERVO) || \
    defined(TEST_TVC) || defined(TEST_LOGGING) || defined(TEST_BARO) || defined(TEST_PYRO)
#include "tests/tests.h"
#endif

// ─────────────────────────────────────────────
//  Pin Definitionsx
// ─────────────────────────────────────────────
static constexpr int LED_PIN = 41;
static constexpr int SERVO1_PIN = 0;
static constexpr int SERVO2_PIN = 2;
static constexpr int PYROPIN = 23;

// ─────────────────────────────────────────────
//  Global Objects
// ─────────────────────────────────────────────

BMI323 imu;
TVC tvc(15.0f);
Servo servoPitch;
Servo servoYaw;
FlightLogger flightlogger;
Baro barometer(Wire2);
float groundPressureHpa = 1010.2f; // will be set to actual ground pressure at init. Setting default to local ground pressure

// ─────────────────────────────────────────────
//  Hardware Init
// ─────────────────────────────────────────────
static void initHardware()
{
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && millis() - t < 2000)
        ;

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    pinMode(PYROPIN, OUTPUT);
    digitalWrite(PYROPIN, LOW);

    Wire.begin();
    Wire2.begin();

    servoPitch.attach(SERVO1_PIN); // Servo 1 is controlling pitch 
    servoYaw.attach(SERVO2_PIN); // Servo 2 is controlling yaw
}

static void indicateError()
{
    // Slowly fade LED in and out to indicate Error
    while (1)
    {
        // fade in
        for (int brightness = 0; brightness <= 255; brightness++)
        {
            analogWrite(LED_PIN, brightness);
            delay(5);
        }

        for (int brightness = 255; brightness >= 0; brightness--)
        {
            analogWrite(LED_PIN, brightness);
            delay(5);
        }
    }
}

static void indicateSuccess()
{

    // Blink LED 3 times to indicate success

    // saves more flash memory using loop
    for (int i = 0; i < 3; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);

        if (i < 2)
        {
            delay(150);
        }
    }

    delay(1000);
}

static void initIMU()
{
    if (!imu.begin())
    {
        // Serial.println("[ERROR] IMU init failed — halting.");
        indicateError();
        while (1)
            ;
    }
    // Serial.println("[OK] IMU ready.");
    indicateSuccess();
}

static void initBaro()
{
    if (!barometer.begin())
    {
        // Serial.println("[ERROR] Barometer init failed — halting.");
        indicateError();
        while (1)
            ;
    }

    // Take a few readings and average to get a stable ground reference
    float pressureSum = 0;
    float temp, alt;
    for (int i = 0; i < 10; i++)
    {
        float p;
        barometer.readAll(p, temp, alt); // uses default 1010.25 — we only care about raw pressure
        pressureSum += p;
        delay(20);
    }
    float groundPressurePa = pressureSum / 10.0f;
    groundPressureHpa = groundPressurePa / 100.0f; // convert Pa → hPa for the formula

    // Serial.printf("[OK] Barometer ready. Ground pressure: %.2f hPa\n", groundPressureHpa);
    indicateSuccess();
}

static void initFlash()
{
    if (!flightlogger.begin())
    {
        indicateError();
        while (1)
            ;
    }
    flightlogger.eraseAll();
    indicateSuccess();
}

static void logFlightData(float ax, float ay, float az, float gx, float gy, float gz, float velocity, float altitude, float servoYawAngle,
                          float servoPitchAngle, FlightState currentState, FlightLogger &flightlogger)
{

    FlightPacket pkt = {
        .timestamp_ms = millis(),
        .altitude_m = altitude,
        .velocity_ms = velocity,
        .accel_x = ax,
        .accel_y = ay,
        .accel_z = az,
        .gyro_x = gx,
        .gyro_y = gy,
        .gyro_z = gz,
        .servoX = servoYawAngle,
        .servoY = servoPitchAngle,
        .state = currentState};

    flightlogger.logPacket(pkt);
}

static void writeServosMicroseconds(Servo &servoPitch, Servo &servoYaw, float servoYawAngleRad, float servoPitchAngleRad)
{
    const int CENTER_Us = 1500;

    // Convert radians directly to microseconds
    // 1000 microseconds corresponds to PI radians (180 degrees)
    const float US_PER_RADIAN = 4000.0f / M_PI;

    const float yawMultiplier = -1.0f;
    const float pitchMultiplier = -1.0f;

    int yawUs = CENTER_Us + (int)(yawMultiplier * servoYawAngleRad * US_PER_RADIAN);
    int pitchUs = CENTER_Us + (int)(pitchMultiplier * servoPitchAngleRad * US_PER_RADIAN);

    const int MIN_US = 1500 - 334; // ~ 15 degrees of gimbal deflection
    const int MAX_US = 1500 + 334; // ~ 15 degrees of gimbal deflection

    yawUs = constrain(yawUs, MIN_US, MAX_US);
    pitchUs = constrain(pitchUs, MIN_US, MAX_US);

    servoYaw.writeMicroseconds(yawUs);
    servoPitch.writeMicroseconds(pitchUs);
}

static float computeHoverslam(float velocity, float accel)
{
    float h = velocity * velocity / (2.0f * (accel)); // assuming accel is in m/s^2
    return h;
}

// ─────────────────────────────────────────────
//  Flight Loop
// ─────────────────────────────────────────────
static void flightLoop()
{

    constexpr float maxGimablAngle = 15.0f; // degrees
    constexpr float SERVO_MAX_DEG_PER_SEC = 600.0f;
    constexpr float CONTROL_LOOP_HZ = 100.0f;
    constexpr float SERVO_MAX_DEG_PER_TICK = SERVO_MAX_DEG_PER_SEC / CONTROL_LOOP_HZ;
    constexpr int BARO_DIVIDER = 2;                 // Update Barometer every 2 ticks (50 Hz)
    constexpr float A_NET_FOR_LANDING_MOTOR = 2.0f; // net acceleration for landing motor, NEEDS TO BE COMPUTED OFFLINE AND FIGURED OUT. COMPUTE IN m/s^2

    constexpr uint32_t LOOP_US = 1000000 / CONTROL_LOOP_HZ; // microseconds per control loop tick

    uint32_t baroCounter = 0;
    uint32_t lastTick = micros();
    float altitude = 0.0f, prevAltitude = 0.0f;
    bool pyroFired = false;
    bool velocityReset = false;
    bool ballisticInit = false;
    float prevServoXcmd = 0.0f, prevServoYcmd = 0.0f;
    uint32_t pyroFireTime = 0;

    float velocity = 0.0f; // simple vertical velocity estimate for ballistic descent detection

    tvc.init(0.1f, 0.01f, 0.05f, maxGimablAngle); // PID gains and max angle (NEEDS to be simulated/tuned!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)

    while (1)
    {
        while (micros() - lastTick < LOOP_US)
        {
        }

        uint32_t now = micros();

        float dt = (now - lastTick) / 1000000.0f; // convert to seconds
        lastTick = now;
        baroCounter++;

        float w, x, y, z;
        float accelx, accely, accelz;
        float gx, gy, gz;

        imu.update();
        imu.getlastAccel(accelx, accely, accelz);
        imu.getlastGyro(gx, gy, gz);
        imu.getQuaternion(w, x, y, z);

        if (baroCounter == BARO_DIVIDER)
        {
            prevAltitude = altitude;
            barometer.getAltitude(altitude, groundPressureHpa);
            baroCounter = 0;
        }

        velocity += (accelz - 1.0f) * 9.81f * dt;

        if (millis() - pyroFireTime >= 750 && pyroFired)
        {
            digitalWrite(PYROPIN, LOW); // Turn off pyro after 750ms pulse
        }

        updateFlightState(accelz, altitude, prevAltitude, dt);

        // ==============================
        // Flight State Switch Statements
        // ==============================

        float servoYawAngle = 0.0f;
        float servoPitchAngle = 0.0f;

        switch (getCurrentState())
        {

        case GROUND_IDLE:
        {
            tvc.disable();
            servoPitch.writeMicroseconds(1500);
            servoYaw.writeMicroseconds(1500);
            servoYawAngle = 0.0f;
            servoPitchAngle = 0.0f;
            prevServoXcmd = 0.0f; // Reset rate limiter baseline
            prevServoYcmd = 0.0f;
            break;
        }
        case POWERED_FLIGHT:
            // TVC active, no pyro  imu.update();
            {
                tvc.update(w, x, y, z, dt);

                servoPitchAngle = tvc.getServoX(); // Pitch in X axis with angle being in radians
                servoYawAngle = tvc.getServoY();   // Yaw in Y axis with angle being in radians

                // clamp delta between new TVC output and previous servo command to max rate of servo
                float deltaYaw = servoYawAngle - prevServoYcmd;
                float deltaPitch = servoPitchAngle - prevServoXcmd;
                deltaYaw = constrain(deltaYaw, -SERVO_MAX_DEG_PER_TICK * (PI / 180.0f), SERVO_MAX_DEG_PER_TICK * (PI / 180.0f));
                deltaPitch = constrain(deltaPitch, -SERVO_MAX_DEG_PER_TICK * (PI / 180.0f), SERVO_MAX_DEG_PER_TICK * (PI / 180.0f));
                servoYawAngle = prevServoYcmd + deltaYaw;
                servoPitchAngle = prevServoXcmd + deltaPitch;

                writeServosMicroseconds(servoPitch, servoYaw, servoYawAngle, servoPitchAngle);

                prevServoXcmd = servoPitchAngle;
                prevServoYcmd = servoYawAngle;

                break;
            }
        case UNPOWERED_FLIGHT:
            // TVC off, no pyro
            {
                tvc.disable();
                servoPitch.writeMicroseconds(1500);
                servoYaw.writeMicroseconds(1500);
                servoYawAngle = 0.0f;
                servoPitchAngle = 0.0f;
                prevServoXcmd = 0.0f; // Reset rate limiter baseline
                prevServoYcmd = 0.0f;
                break;
            }
        case BALLISTIC_DESCENT:
            // TVC off, no pyro.
            {
                if (!ballisticInit)
                {
                    velocityReset = true;
                    ballisticInit = true;
                }

                if (velocityReset)
                {
                    velocity = 0;
                    velocityReset = false;
                }

                float currentAltitude = computeHoverslam(velocity, A_NET_FOR_LANDING_MOTOR);

                if (!pyroFired && prevAltitude < currentAltitude)
                {
                    digitalWrite(PYROPIN, HIGH); // Fire pyro
                    pyroFired = true;
                    pyroFireTime = millis();
                    tvc.init(0.1f, 0.01f, 0.05f, maxGimablAngle); // Re-initialize PID for powered descent (NEEDS TO BE TUNED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)
                    setFlightState(POWERED_DESCENT);              // Transition to powered descent after firing pyro
                }

                break;
            }
        case POWERED_DESCENT:
        {

            tvc.update(w, x, y, z, dt);

            servoPitchAngle = tvc.getServoX(); // Pitch in X axis with angle being in radians
            servoYawAngle = tvc.getServoY();   // Yaw in Y axis with angle being in radians

            // clamp delta between new TVC output and previous servo command to max rate of servo

            float deltaYaw = servoYawAngle - prevServoYcmd;
            float deltaPitch = servoPitchAngle - prevServoXcmd;
            deltaYaw = constrain(deltaYaw, -SERVO_MAX_DEG_PER_TICK * (PI / 180.0f), SERVO_MAX_DEG_PER_TICK * (PI / 180.0f));
            deltaPitch = constrain(deltaPitch, -SERVO_MAX_DEG_PER_TICK * (PI / 180.0f), SERVO_MAX_DEG_PER_TICK * (PI / 180.0f));
            servoYawAngle = prevServoYcmd + deltaYaw;
            servoPitchAngle = prevServoXcmd + deltaPitch;

            writeServosMicroseconds(servoPitch, servoYaw, servoYawAngle, servoPitchAngle);
            prevServoXcmd = servoPitchAngle;
            prevServoYcmd = servoYawAngle;

            break;
        }
        case LANDING:
        {
            tvc.disable();
            servoPitch.writeMicroseconds(1500); // Center position
            servoYaw.writeMicroseconds(1500);   // Center position
            servoYawAngle = 0.0f;
            servoPitchAngle = 0.0f;
            break;
        }
        }

        logFlightData(accelx, accely, accelz, gx, gy, gz, velocity, altitude, servoYawAngle, servoPitchAngle, getCurrentState(),
                      flightlogger);
    }
}


// ─────────────────────────────────────────────
//  Entry Point
// ─────────────────────────────────────────────

int main()
{
    initHardware();

#if defined(TEST_I2C_SCAN)
    test_i2cScan(2);

#elif defined(TEST_SPI_SCAN)
    test_spiScan();

#elif defined(TEST_IMU_ACCEL)
    initIMU();
    test_imuAccel(imu);

#elif defined(TEST_IMU_GYRO)
    initIMU();
    test_imuGyro(imu);

#elif defined(TEST_IMU_QUATERNION)
    initIMU();
    test_imuQuaternion(imu);

#elif defined(TEST_SERVO)
    test_servo(servoPitch, servoYaw);

#elif defined(TEST_TVC)
    initIMU();
    test_tvc(imu, tvc, servoPitch, servoYaw);

#elif defined(TEST_LOGGING)
    initIMU();
    test_logging(imu, flightlogger);

#elif defined(TEST_BARO)
    initBaro();
    testBaro(barometer);

#elif defined(TEST_PYRO)
    testPyroChannel(pyroPin);

#else
    // initIMU();
    // initBaro();
    // initFlash();
    // flightLoop();

    writeServosMicroseconds(servoPitch, servoYaw, -3.14159f, -3.14159f); 

#endif

    return 0;
}