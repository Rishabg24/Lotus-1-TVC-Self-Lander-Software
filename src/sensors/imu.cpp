// BMI323 IMU driver with Madgwick sensor fusion for TVC rocketry
#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include <cstdint>
#include <vector>
#include <cmath>

BMI323::BMI323(uint8_t address) : addr(address)
{
    beta = 0.033f;       // Madgwick filter gain for IMU (original paper recommendation)
    sampleRate = 100.0f; // Sample rate in Hz - must match actual ODR
}

bool BMI323::begin()
{
    Wire.begin(); // initialize I2C
    delay(10);    // Wait for sensor to be ready after power-on

    uint16_t chipID = 0;

    // BMI323 requires 2 dummy bytes on I2C read
    // First read might fail if sensor just powered on
    readReg16(REG_CHIP_ID, &chipID);
    delay(5);

    // Read chip ID (lower byte should be 0x43)
    if (!readReg16(REG_CHIP_ID, &chipID))
    {
        return false;
    }

    if ((chipID & 0xFF) != 0x43)
    {
        return false; // wrong chip ID
    }

    // Small delay before configuration
    delay(10);

    configureAccelerometer();
    delay(5);
    configureGyroscope();
    delay(5);

    // Set scale factors based on configured ranges
    // For ±4G range: sensitivity = 8192 LSB/g
    // Scale = range / 32768
    accelScale = 4.0f / 8192.0f; // ±4G range, sensitivity 8192 LSB/g

    // For ±125 dps range: sensitivity = 262.144 LSB/(°/s)
    // Scale = range / sensitivity = 125 / 262.144
    // Convert to rad/s: multiply by (π/180)
    gyroScale = (125.0f / 262.144f) * (M_PI / 180.0f); // ±125 dps range to rad/s

    // Initialize quaternion to identity (no rotation)
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    return true;
}

bool BMI323::writeReg16(uint8_t reg, uint16_t value)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value & 0xFF);        // LSB first
    Wire.write((value >> 8) & 0xFF); // MSB second
    return (Wire.endTransmission() == 0);
}

bool BMI323::readReg16(uint8_t reg, uint16_t *value)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
    {
        return false;
    }

    // BMI323 I2C protocol: 2 dummy bytes + 2 data bytes
    Wire.requestFrom(addr, (uint8_t)4);

    // Discard 2 dummy bytes
    if (Wire.available() < 4)
        return false;
    Wire.read(); // dummy byte 1
    Wire.read(); // dummy byte 2

    // Read actual data (LSB first)
    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();

    *value = (uint16_t)lsb | ((uint16_t)msb << 8);
    return true;
}

void BMI323::configureAccelerometer()
{
    // Configure: Normal mode, 100Hz ODR, ±4G range, ODR/2 bandwidth
    uint16_t accConf = ACC_MODE_NORMAL | ACC_ODR_100HZ | ACC_RANGE_4G | ACC_BW_ODR_DIV_2;
    writeReg16(REG_ACC_CONF, accConf);
}

void BMI323::configureGyroscope()
{
    // Configure: Normal mode, 100Hz ODR, ±125dps range, ODR/2 bandwidth
    uint16_t gyrConf = GYR_MODE_NORMAL | GYR_ODR_100HZ | GYR_RANGE_125DPS | GYR_BW_ODR_DIV_2;
    writeReg16(REG_GYR_CONF, gyrConf);
}

void BMI323::readAccelerometer(float &ax, float &ay, float &az)
{
    uint16_t rawData[3];

    readReg16(REG_ACC_DATA_X, &rawData[0]);
    readReg16(REG_ACC_DATA_Y, &rawData[1]);
    readReg16(REG_ACC_DATA_Z, &rawData[2]);

    // Convert to signed 16-bit values
    int16_t rawAx = (int16_t)rawData[0];
    int16_t rawAy = (int16_t)rawData[1];
    int16_t rawAz = (int16_t)rawData[2];

    // Apply scale factors to get acceleration in g
    ax = rawAx * accelScale;
    ay = rawAy * accelScale;
    az = rawAz * accelScale;
}

void BMI323::readGyroscope(float &gx, float &gy, float &gz)
{
    uint16_t rawData[3];

    readReg16(REG_GYR_DATA_X, &rawData[0]);
    readReg16(REG_GYR_DATA_Y, &rawData[1]);
    readReg16(REG_GYR_DATA_Z, &rawData[2]);

    // Convert to signed 16-bit values
    int16_t rawGx = (int16_t)rawData[0];
    int16_t rawGy = (int16_t)rawData[1];
    int16_t rawGz = (int16_t)rawData[2];

    // Apply scale factors to get angular rate in rad/s
    gx = rawGx * gyroScale;
    gy = rawGy * gyroScale;
    gz = rawGz * gyroScale;
}

void BMI323::readTemperature(float &temp)
{
    uint16_t rawData;
    readReg16(REG_TEMP_DATA, &rawData);

    int16_t rawTemp = (int16_t)rawData;

    // BMI323 temperature: 0 LSB at 23°C, 512 LSB/K
    temp = (rawTemp / 512.0f) + 23.0f;
}

void BMI323::madgwickUpdate(float ax, float ay, float az,
                            float gx, float gy, float gz)
{
    // Madgwick AHRS algorithm - 6DOF (no magnetometer)
    // Reference: "An efficient orientation filter for inertial and
    // inertial/magnetic sensor arrays" by Sebastian Madgwick
    // https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

    // Normalize accelerometer measurement
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm < 0.001f)
        return; // Avoid division by zero (free fall or invalid data)

    float recip_norm = 1.0f / acc_norm;
    ax *= recip_norm;
    ay *= recip_norm;
    az *= recip_norm;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    // Gradient descent algorithm corrective step
    // This computes the error between measured gravity and predicted gravity
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    // Normalize gradient to prevent excessive corrections
    recip_norm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recip_norm;
    s1 *= recip_norm;
    s2 *= recip_norm;
    s3 *= recip_norm;

    // Compute rate of change of quaternion from gyroscope
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Apply feedback step (subtract gradient-based correction from gyro rate)
    // This is where the accelerometer corrects gyro drift
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;

    // Integrate rate of change to yield quaternion
    float dt = 1.0f / sampleRate;
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalize quaternion
    recip_norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm;
    q1 *= recip_norm;
    q2 *= recip_norm;
    q3 *= recip_norm;
}

void BMI323::getQuaternion(float &w, float &x, float &y, float &z)
{
    w = q0;
    x = q1;
    y = q2;
    z = q3;
}

void BMI323::update()
{

    readAccelerometer(last_ax, last_ay, last_az);
    readGyroscope(last_gx, last_gy, last_gz);

    // Run Madgwick filter with gyro in rad/s and accel in g
    madgwickUpdate(last_ax, last_ay, last_az, last_gx, last_gy, last_gz);
}

void BMI323::getlastAccel(float &ax, float &ay, float &az){
    ax = last_ax;
    ay = last_ay;
    az = last_az;
}

void BMI323::getlastGyro(float &gx, float &gy, float &gz){
    gx = last_gx;
    gy = last_gy;
    gz = last_gz;
}