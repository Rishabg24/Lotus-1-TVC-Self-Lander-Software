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
    if (type == 1){
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
    while (1);
    }
    else if(type == 2){
        Serial.println("── I2C Bus 2 Scan ──");

        for (TwoWire *bus : {&Wire, &Wire1, &Wire2})
    {
        bus->begin();
        for (uint8_t addr = 1; addr < 127; addr++)
        {
            bus->beginTransmission(addr);
            if (bus->endTransmission() == 0)
                Serial.printf("  Found 0x%02X on Wire%d\n", addr, bus == &Wire ? 0 : bus == &Wire1 ? 1 : 2);
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
        float ax, ay, az;
        imu.readAccelerometer(ax, ay, az);
        Serial.printf("Accel  X: %6.3f  Y: %6.3f  Z: %6.3f  g\n", ax, ay, az);
        delay(100);
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
        delay(100);
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
    tvc.init(1.0f, 0.0f, 0.1f, 15.0f);

    uint32_t lastTime = micros();

    while (1)
    {
        uint32_t now = micros();
        float dt = (now - lastTime) / 1e6f;
        lastTime = now;

        imu.update();
        float w, x, y, z;
        imu.getQuaternion(w, x, y, z);

        tvc.update(w, x, y, z, dt);

        float servoX = tvc.getServoX();
        float servoY = tvc.getServoY();

        servo1.write(90 + (int)servoX);
        servo2.write(90 + (int)servoY);

        Serial.printf("ServoX: %6.2f°  ServoY: %6.2f°\n", servoX, servoY);
        delay(10);
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