#include "tests.h"
#include "../sensors/baro.h"
#include <SPI.h>
#include <Wire.h>

// ─────────────────────────────────────────────
//  Test Functions
//  Each loops forever — reset or Ctrl+C to exit.
//  Hardware objects are passed in from main.cpp;
//  nothing is declared here to avoid duplicate globals.
// ─────────────────────────────────────────────

// Scan the I2C bus and print every responding address.
// Useful when a sensor isn't responding — confirms wiring before debugging code.
void test_i2cScan(int type)
{
    if (type == 1)
    {
        Serial.println("── I2C Scan ──");
        for (uint8_t addr = 1; addr < 127; addr++)
        {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0)
            {
                Serial.print("  Found device at 0x");
                Serial.println(addr, HEX);
            }
        }
        Serial.println("── Scan done ──");
        while (1)
            ;
    }
    else if (type == 2)
    {
        Serial.println("── I2C Bus 2 Scan ──");

        for (TwoWire *bus : {&Wire, &Wire1, &Wire2})
        {
            bus->begin();
            for (uint8_t addr = 1; addr < 127; addr++)
            {
                bus->beginTransmission(addr);
                if (bus->endTransmission() == 0)
                    Serial.printf("  Found 0x%02X on Wire%d\n", addr, bus == &Wire ? 0 : bus == &Wire1 ? 1
                                                                                                       : 2);
            }
        }
    }
}

void test_spiScan()
{
    Serial.println("── SPI Flash Scan ──");

    uint8_t csPins[] = {9, 10};

    SPI.begin();

    for (uint8_t i = 0; i < sizeof(csPins); i++)
    {
        uint8_t cs = csPins[i];
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);
    }

    for (uint8_t i = 0; i < sizeof(csPins); i++)
    {
        uint8_t cs = csPins[i];

        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWrite(cs, LOW);

        SPI.transfer(0x9F);
        uint8_t b1 = SPI.transfer(0x00);
        uint8_t b2 = SPI.transfer(0x00);
        uint8_t b3 = SPI.transfer(0x00);

        digitalWrite(cs, HIGH);
        SPI.endTransaction();

        Serial.printf("CS pin %d → %02X %02X %02X", cs, b1, b2, b3);

        if (b1 == 0xEF && b2 == 0x40 && b3 == 0x16)
            Serial.println("  ✓ W25Q32 found");
        else if (b1 == 0xFF && b2 == 0xFF && b3 == 0xFF)
            Serial.println("  ✗ No response (check wiring)");
        else if (b1 == 0x00 && b2 == 0x00 && b3 == 0x00)
            Serial.println("  ✗ All zeros (CS/MISO shorted or floating)");
        else
            Serial.println("  ? Unknown device");
    }

    Serial.println("── Scan done ──");
    while (1)
        ;
}

void test_imuAccel(BMI323 &imu)
{
    Serial.println("── IMU Accelerometer Test (10 Hz) ──");
    while (1)
    {
        imu.update();
        float ax, ay, az;
        imu.getlastAccel(ax, ay, az); // ← reads remapped values
        Serial.printf("Accel  X: %6.3f  Y: %6.3f  Z: %6.3f  g\n", ax, ay, az);
        delay(300);
    }
}

void test_imuGyro(BMI323 &imu)
{
    Serial.println("── IMU Gyroscope Test (10 Hz) ──");
    while (1)
    {
        float gx, gy, gz;
        imu.readGyroscope(gx, gy, gz);
        Serial.printf("Gyro   X: %6.3f  Y: %6.3f  Z: %6.3f  rad/s\n", gx, gy, gz);
        delay(300);
    }
}

void testBaro(Baro &barometer)
{
    Serial.println("-- Barometer Test (10 Hz) --");
    while (1)
    {
        float pressure, temperature, altitude;
        barometer.readAll(pressure, temperature, altitude);

        Serial.printf("Pressure: %7.2f Pa  Temperature: %5.2f °C  Altitude: %6.2f m\n",
                      pressure, temperature, altitude);
        delay(100);
    }
}

void test_imuQuaternion(BMI323 &imu)
{
    Serial.println("── IMU Quaternion Test (4 Hz) ──");
    while (1)
    {
        imu.update();
        float w, x, y, z;
        imu.getQuaternion(w, x, y, z);
        Serial.printf("Quat   W: %6.4f  X: %6.4f  Y: %6.4f  Z: %6.4f\n", w, x, y, z);
        delay(250);
    }
}

void test_servo(Servo &servo1, Servo &servo2)
{
    Serial.println("── Servo Test ──");
    while (1)
    {
        for (int angle = 0; angle <= 180; angle += 5)
        {
            servo1.write(angle);
            servo2.write(180 - angle);
            Serial.printf("Servo1: %d°  Servo2: %d°\n", angle, 180 - angle);
            delay(50);
        }
        for (int angle = 180; angle >= 0; angle -= 5)
        {
            servo1.write(angle);
            servo2.write(180 - angle);
            Serial.printf("Servo1: %d°  Servo2: %d°\n", angle, 180 - angle);
            delay(50);
        }
    }
}

void testPyroChannel(int pyroPin)
{
    Serial.println("── Pyro Channel Test ──");
    pinMode(pyroPin, OUTPUT);
    while (1)
    {
        digitalWrite(pyroPin, HIGH);
        Serial.println("Pyro channel ON");
        delay(1000);
        digitalWrite(pyroPin, LOW);
        Serial.println("Pyro channel OFF");
        delay(1000);
    }
}
void test_tvc(BMI323 &imu, TVC &tvc, Servo &servo1, Servo &servo2)
{
    Serial.println("── TVC Test ──");
    constexpr float maxGimbalAngleRad = 16.0f * (M_PI / 180.0f);

    // Note: These gains (0.1, 0.01, 0.05) are quite low for radians.
    // You may need to increase them once the jitter is gone to get strong deflection!
    tvc.init(4.5f, 0.00f, 0.01f, maxGimbalAngleRad);

    // === MATCH FLIGHT LOOP TIMING (100 Hz) ===
    constexpr float CONTROL_LOOP_HZ = 100.0f;
    constexpr uint32_t LOOP_US = 1000000 / CONTROL_LOOP_HZ;

    uint32_t lastTick = micros();
    uint32_t lastPrintTime = millis();

    const int CENTER_Us = 1500;
    const float US_PER_RADIAN = 4000.0f / M_PI;

    // Dynamically compute limits based on your max gimbal angle
    const int maxDeflectionUs = (int)(maxGimbalAngleRad * US_PER_RADIAN);
    const int MIN_US = CENTER_Us - maxDeflectionUs; // Will be exactly 1500 - 355
    const int MAX_US = CENTER_Us + maxDeflectionUs; // Will be exactly 1500 + 355

    while (1)
    {
        // 1. Enforce strict 100Hz timing
        while (micros() - lastTick < LOOP_US)
        {
            // Wait for the next 10ms frame
        }

        uint32_t now = micros();
        float dt = (now - lastTick) / 1000000.0f; // dt should confidently be ~0.01
        lastTick = now;

        // 2. Read IMU at exactly 100Hz
        imu.update();

        float w, x, y, z;
        imu.getQuaternion(w, x, y, z);

        tvc.update(w, x, y, z, dt);

        float servoXRad = tvc.getServoX();
        float servoYRad = tvc.getServoY();

        int pitchUs = CENTER_Us + (int)(-1.0f * servoXRad * US_PER_RADIAN);
        int yawUs = CENTER_Us + (int)(-1.0f * servoYRad * US_PER_RADIAN);

        pitchUs = constrain(pitchUs, MIN_US, MAX_US);
        yawUs = constrain(yawUs, MIN_US, MAX_US);

        // 3. Write updates to hardware at exactly 100Hz
        servo1.writeMicroseconds(yawUs); // just for testing
        servo2.writeMicroseconds(pitchUs);

        // Print at 5Hz
        if (millis() - lastPrintTime >= 200)
        {
            lastPrintTime = millis();
            float gx, gy, gz;
            imu.getlastGyro(gx, gy, gz);

            Serial.printf("Q: w=%.3f x=%.3f y=%.3f z=%.3f\n", w, x, y, z);
            Serial.printf("TVC Rads -> X: %.4f rad | Y: %.4f rad\n", servoXRad, servoYRad);
            Serial.printf("Servo PWM -> Pitch: %d us | Yaw: %d us\n", pitchUs, yawUs);
            Serial.printf("Gyro -> x: %.4f | y: %.4f | z: %.4f\n\n", gx, gy, gz);
        }
    }
}

void test_logging(BMI323 &imu, FlightLogger &flightlogger)
{
    if (!flightlogger.begin())
    {
        Serial.println("[ERROR] FlightLogger init failed — halting.");
        return;
    }

    float x, y, z, gx, gy, gz;
    imu.readAccelerometer(x, y, z);
    imu.readGyroscope(gx, gy, gz);

    FlightPacket pkt = {
        .timestamp_ms = millis(),
        .altitude_m = 100.0f,
        .velocity_ms = 50.0f,
        .accel_x = x,
        .accel_y = y,
        .accel_z = z,
        .gyro_x = gx,
        .gyro_y = gy,
        .gyro_z = gz,
        .servoX = 10.0f,
        .servoY = -5.0f,
        .state = 1};

    flightlogger.eraseAll();

    if (flightlogger.logPacket(pkt))
        Serial.println("[OK] Packet logged successfully.");
    else
        Serial.println("[ERROR] Failed to log packet.");

    delay(1000);

    Serial.printf("Logged %u packets.\n", flightlogger.packetCount());
    Serial.println("\n── Dump ──");
    flightlogger.dump();
}