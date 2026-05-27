#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <cstdint>

// I2C Address
#define BMI323_ADDRESS 0x68

// Registers (16-bit addressing, but accessed as 8-bit address)
#define REG_CHIP_ID 0x00 // Expected value: 0x0043 (only lower byte 0x43 valid)
#define REG_ERR_REG 0x01
#define REG_STATUS 0x02

// Sensor data registers (16-bit words)
#define REG_ACC_DATA_X 0x03
#define REG_ACC_DATA_Y 0x04
#define REG_ACC_DATA_Z 0x05
#define REG_GYR_DATA_X 0x06
#define REG_GYR_DATA_Y 0x07
#define REG_GYR_DATA_Z 0x08
#define REG_TEMP_DATA 0x09

// Configuration registers
#define REG_ACC_CONF 0x20
#define REG_GYR_CONF 0x21

// -------- ACCEL CONFIG BITS (16-bit register) --------------
// ACC_CONF layout: [15:14]=mode, [13:12]=avg, [11:8]=unused, [7]=bw, [6:5]=unused, [4:0]=range+odr
#define ACC_MODE_SHIFT 12
#define ACC_MODE_SUSPEND (0x0 << ACC_MODE_SHIFT)
#define ACC_MODE_LOW_POWER (0x3 << ACC_MODE_SHIFT)
#define ACC_MODE_NORMAL (0x4 << ACC_MODE_SHIFT)
#define ACC_MODE_HIGH_PERF (0x7 << ACC_MODE_SHIFT)

#define ACC_ODR_SHIFT 0
#define ACC_ODR_12_5HZ (0x05 << ACC_ODR_SHIFT) // ODR = 12.5Hz
#define ACC_ODR_25HZ (0x06 << ACC_ODR_SHIFT)   // ODR = 25Hz
#define ACC_ODR_50HZ (0x07 << ACC_ODR_SHIFT)   // ODR = 50Hz
#define ACC_ODR_100HZ (0x08 << ACC_ODR_SHIFT)  // ODR = 100Hz
#define ACC_ODR_200HZ (0x09 << ACC_ODR_SHIFT)  // ODR = 200Hz

#define ACC_RANGE_SHIFT 4
#define ACC_RANGE_2G (0x0 << ACC_RANGE_SHIFT)
#define ACC_RANGE_4G (0x1 << ACC_RANGE_SHIFT)
#define ACC_RANGE_8G (0x2 << ACC_RANGE_SHIFT)
#define ACC_RANGE_16G (0x3 << ACC_RANGE_SHIFT)

#define ACC_BW_SHIFT 7
#define ACC_BW_ODR_DIV_2 (0x0 << ACC_BW_SHIFT)
#define ACC_BW_ODR_DIV_4 (0x1 << ACC_BW_SHIFT)

// -------- GYRO CONFIG BITS (16-bit register) ---------------
#define GYR_MODE_SHIFT 12
#define GYR_MODE_SUSPEND (0x0 << GYR_MODE_SHIFT)
#define GYR_MODE_LOW_POWER (0x3 << GYR_MODE_SHIFT)
#define GYR_MODE_NORMAL (0x4 << GYR_MODE_SHIFT)
#define GYR_MODE_HIGH_PERF (0x7 << GYR_MODE_SHIFT)

#define GYR_ODR_SHIFT 0
#define GYR_ODR_12_5HZ (0x05 << GYR_ODR_SHIFT)
#define GYR_ODR_25HZ (0x06 << GYR_ODR_SHIFT)
#define GYR_ODR_50HZ (0x07 << GYR_ODR_SHIFT)
#define GYR_ODR_100HZ (0x08 << GYR_ODR_SHIFT)
#define GYR_ODR_200HZ (0x09 << GYR_ODR_SHIFT)

#define GYR_RANGE_SHIFT 4
#define GYR_RANGE_125DPS (0x0 << GYR_RANGE_SHIFT)
#define GYR_RANGE_250DPS (0x1 << GYR_RANGE_SHIFT)
#define GYR_RANGE_500DPS (0x2 << GYR_RANGE_SHIFT)
#define GYR_RANGE_1000DPS (0x3 << GYR_RANGE_SHIFT)
#define GYR_RANGE_2000DPS (0x4 << GYR_RANGE_SHIFT)

#define GYR_BW_SHIFT 7
#define GYR_BW_ODR_DIV_2 (0x0 << GYR_BW_SHIFT)
#define GYR_BW_ODR_DIV_4 (0x1 << GYR_BW_SHIFT)

class BMI323
{
public:
    BMI323(uint8_t address = BMI323_ADDRESS);

    bool begin();

    void readAccelerometer(float &ax, float &ay, float &az);
    void readGyroscope(float &gx, float &gy, float &gz);
    void readTemperature(float &temp);

    // This will read sensors AND run Madgwick update
    void update();

    // Retrieve orientation from quaternion
    void getQuaternion(float &w, float &x, float &y, float &z);

    void getlastAccel(float &ax, float &ay, float &az);
    void getlastGyro(float &gx, float &gy, float &gz);

private:
    uint8_t addr;

    // Scale factors (set after reading range configuration)
    float accelScale;
    float gyroScale;

    // Quaternion state
    float q0, q1, q2, q3;

    // Madgwick parameters
    float beta;
    float sampleRate;

    // Low-level register IO
    bool writeReg16(uint8_t reg, uint16_t value);
    bool readReg16(uint8_t reg, uint16_t *value);

    // Configuration helpers
    void configureAccelerometer();
    void configureGyroscope();

    // Sensor fusion
    void madgwickUpdate(float ax, float ay, float az,
                        float gx, float gy, float gz);

    float last_ax, last_ay, last_az; // Store last raw accel readings for access outside of update
    float last_gx, last_gy, last_gz; // Store last raw gyro readings for potential debugging
};