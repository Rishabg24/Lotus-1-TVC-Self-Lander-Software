#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <cstdint>

#define BARO_ADDRESS 0x76 //  I2C address for barometric sensor

#define REG_CHIP_ID_BARO 0x00
#define REG_ERR_REF_BARO 0x02
#define REG_CONF 0x1F
#define REG_STATUS_BARO 0x03

// Pressure data split and stored in three consecutive registers
#define REG_DATA_0 0X04
#define REG_DATA_1 0X05
#define REG_DATA_2 0X06

// Temperature data split and stored in three consecutive registers
#define REG_DATA_3 0X07
#define REG_DATA_4 0X08
#define REG_DATA_5 0X09

#define SENSORTIME_0 0x0C
#define SENSORTIME_1 0x0D
#define SENSORTIME_2 0x0E

#define REG_FIFO_LENGTH_0 0x12
#define REG_FIFO_LENGTH_1 0x13
#define REG_FIFO_DATA 0x14
#define REG_FIFO_WTM_0 0x15
#define REG_FIFO_WTM_1 0x16
#define REG_FIFO_CONFIG_1 0x17
#define REG_FIFO_CONFIG_2 0x18

#define REG_CALIB_DATA 0x31 // Start of calibration coefficients

// Command register
#define REG_CMD 0x7E

// -------- PWR_CTRL BIT DEFINITIONS --------
// Enable pressure and temperature sensors
#define PWE_PRESS_EN (1 << 0)
#define PWE_TEMP_EN (1 << 1)
#define REG_PWR_CTRL 0x1B
// Power modes (bits 5:4)
#define PWR_MODE_SHIFT 4
#define PWR_MODE_SLEEP (0x0 << PWR_MODE_SHIFT)
#define PWR_MODE_FORCED (0x1 << PWR_MODE_SHIFT) // Also 0x2
#define PWR_MODE_NORMAL (0x3 << PWR_MODE_SHIFT)

// -------- OSR (OVERSAMPLING) BIT DEFINITIONS --------
// Pressure oversampling (bits 2:0)
#define REG_OSR     0x1C
#define OSR_P_SHIFT 0
#define OSR_P_X1 (0x0 << OSR_P_SHIFT) // No oversampling
#define OSR_P_X2 (0x1 << OSR_P_SHIFT)
#define OSR_P_X4 (0x2 << OSR_P_SHIFT)
#define OSR_P_X8 (0x3 << OSR_P_SHIFT)
#define OSR_P_X16 (0x4 << OSR_P_SHIFT)
#define OSR_P_X32 (0x5 << OSR_P_SHIFT)

// Temperature oversampling (bits 5:3)
#define OSR_T_SHIFT 3
#define OSR_T_X1 (0x0 << OSR_T_SHIFT)
#define OSR_T_X2 (0x1 << OSR_T_SHIFT)
#define OSR_T_X4 (0x2 << OSR_T_SHIFT)
#define OSR_T_X8 (0x3 << OSR_T_SHIFT)
#define OSR_T_X16 (0x4 << OSR_T_SHIFT)
#define OSR_T_X32 (0x5 << OSR_T_SHIFT)

// -------- ODR (OUTPUT DATA RATE) BIT DEFINITIONS --------
// ODR selection (bits 4:0) - subdivision of 200Hz base
#define REG_ODR     0x1D
#define ODR_200_HZ 0x00  // 200 Hz
#define ODR_100_HZ 0x01  // 100 Hz
#define ODR_50_HZ 0x02   // 50 Hz
#define ODR_25_HZ 0x03   // 25 Hz
#define ODR_12_5_HZ 0x04 // 12.5 Hz
#define ODR_6_25_HZ 0x05 // 6.25 Hz
#define ODR_3_1_HZ 0x06  // 3.1 Hz
#define ODR_1_5_HZ 0x07  // 1.5 Hz

// -------- CONFIG (IIR FILTER) BIT DEFINITIONS --------
// IIR filter coefficient (bits 3:1)
#define IIR_FILTER_SHIFT 1
#define IIR_FILTER_OFF (0x0 << IIR_FILTER_SHIFT) // Bypass
#define IIR_FILTER_COEF_1 (0x1 << IIR_FILTER_SHIFT)
#define IIR_FILTER_COEF_3 (0x2 << IIR_FILTER_SHIFT)
#define IIR_FILTER_COEF_7 (0x3 << IIR_FILTER_SHIFT)
#define IIR_FILTER_COEF_15 (0x4 << IIR_FILTER_SHIFT)
#define IIR_FILTER_COEF_31 (0x5 << IIR_FILTER_SHIFT)
#define IIR_FILTER_COEF_63 (0x6 << IIR_FILTER_SHIFT)
#define IIR_FILTER_COEF_127 (0x7 << IIR_FILTER_SHIFT)

// -------- COMMAND DEFINITIONS --------
#define CMD_NOP 0x00
#define CMD_FIFO_FLUSH 0xB0
#define CMD_SOFT_RESET 0xB6

struct BMP390_CalibData
{
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
};

struct BMP390CalibDataQ
{
    double par_t1, par_t2, par_t3;
    double par_p1, par_p2, par_p3, par_p4;
    double par_p5, par_p6, par_p7, par_p8;
    double par_p9, par_p10, par_p11;
};

class Baro
{
public:
    Baro(TwoWire &wire = Wire, uint8_t address = BARO_ADDRESS);

    bool begin();
    void readPressure(float &pressure);
    void readTemperature(float &temperature);
    void getAltitude(float &altitude, float seaLevelhPa = 1010.2f);
    void readAll(float &pressure, float &temperature, float &altitude, float seaLevelPressure = 1010.2f);

private:
    TwoWire &_wire;
    uint8_t addr;

    bool writeReg8(uint8_t reg, uint8_t value);
    bool readReg8(uint8_t reg, uint8_t *value);
    bool readReg24(uint8_t reg, uint32_t *value);

    void readCalibrationData();
    void configureSensor();

    float compensatePressure(uint32_t rawPress);
    float compensateTemperature(uint32_t rawTemp);

    BMP390_CalibData calibData;
    BMP390CalibDataQ calibDataQ; // converted floats - used in compensation calculations

    double t_lin; /// linear temperature for pressure compensation
};

