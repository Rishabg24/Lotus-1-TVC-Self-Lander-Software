// Get altitude from barometric pressure sensor (BMP390)
#include <Arduino.h>
#include <Wire.h>
#include "baro.h"

Baro::Baro(TwoWire &wire, uint8_t address) : _wire(wire), addr(address), t_lin(0.0f)
{
}

bool Baro::begin()
{
    _wire.begin();
    delay(10); // Wait for sensor to be ready after power-on

    uint8_t chipID = 0;

    if (!readReg8(REG_CHIP_ID_BARO, &chipID))
    {
        return false;
    }

    if (chipID != 0x60)
    {
        return false; // wrong chip ID
    }

    writeReg8(REG_CMD, CMD_SOFT_RESET);
    delay(10); // wait for reset to complete

    readCalibrationData();

    // // Add this block:
    // Serial.printf("par_t1=%u par_t2=%u par_t3=%d\n", calibData.par_t1, calibData.par_t2, calibData.par_t3);
    // Serial.printf("par_p1=%d par_p2=%d par_p3=%d par_p4=%d\n", calibData.par_p1, calibData.par_p2, calibData.par_p3, calibData.par_p4);
    // Serial.printf("par_p5=%u par_p6=%u par_p7=%d par_p8=%d\n", calibData.par_p5, calibData.par_p6, calibData.par_p7, calibData.par_p8);
    // Serial.printf("par_p9=%d par_p10=%d par_p11=%d\n", calibData.par_p9, calibData.par_p10, calibData.par_p11);

    // Also dump the raw bytes so we can see what the sensor actually sent:
    Serial.print("raw calib bytes: ");
    _wire.beginTransmission(addr);
    _wire.write(REG_CALIB_DATA);
    _wire.endTransmission(false);
    _wire.requestFrom(addr, (uint8_t)21);
    for (int i = 0; i < 21 && _wire.available(); i++)
        Serial.printf("%02X ", _wire.read());
    Serial.println();

    configureSensor();
    delay(50);

    return true;
}

bool Baro::writeReg8(uint8_t reg, uint8_t value)
{
    _wire.beginTransmission(addr);
    _wire.write(reg);
    _wire.write(value);

    return (_wire.endTransmission() == 0);
}

bool Baro::readReg8(uint8_t reg, uint8_t *value)
{
    _wire.beginTransmission(addr);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0)
    {
        return false;
    }

    _wire.requestFrom(addr, (uint8_t)1);
    if (_wire.available())
    {
        *value = _wire.read();
        return true;
    }

    return false;
}

bool Baro::readReg24(uint8_t reg, uint32_t *value)
{
    _wire.beginTransmission(addr);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;

    _wire.requestFrom(addr, (uint8_t)3);
    if (_wire.available() < 3) return false;

    uint8_t xlsb  = _wire.read();
    uint8_t lsb = _wire.read();
    uint8_t msb  = _wire.read();

    *value = (((uint32_t)msb << 16) | ((uint32_t)lsb << 8) | xlsb);
    return true;
}

void Baro::readCalibrationData()
{
    _wire.beginTransmission(addr);
    _wire.write(REG_CALIB_DATA);
    _wire.endTransmission(false);
    _wire.requestFrom(addr, (uint8_t)21);

    uint8_t buf[21];
    for (int i = 0; i < 21 && _wire.available(); i++)
        buf[i] = _wire.read();

    // All multi-byte coefficients are little-endian (LSB first)
    calibData.par_t1 = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    calibData.par_t2 = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    calibData.par_t3 = (int8_t)buf[4];

    calibData.par_p1 = (int16_t)((uint16_t)buf[5] | ((uint16_t)buf[6] << 8));
    calibData.par_p2 = (int16_t)((uint16_t)buf[7] | ((uint16_t)buf[8] << 8));
    calibData.par_p3 = (int8_t)buf[9];
    calibData.par_p4 = (int8_t)buf[10];
    calibData.par_p5 = (uint16_t)buf[11] | ((uint16_t)buf[12] << 8);
    calibData.par_p6 = (uint16_t)buf[13] | ((uint16_t)buf[14] << 8);
    calibData.par_p7 = (int8_t)buf[15];
    calibData.par_p8 = (int8_t)buf[16];
    calibData.par_p9 = (int16_t)((uint16_t)buf[17] | ((uint16_t)buf[18] << 8));
    calibData.par_p10 = (int8_t)buf[19];
    calibData.par_p11 = (int8_t)buf[20];

    calibDataQ.par_t1  = (double)calibData.par_t1  / 0.00390625;          // NVM / 2^-8  = NVM * 256
    calibDataQ.par_t2  = (double)calibData.par_t2  / 1073741824.0;        // NVM / 2^30
    calibDataQ.par_t3  = (double)calibData.par_t3  / 281474976710656.0;   // NVM / 2^48

    calibDataQ.par_p1  = ((double)calibData.par_p1  - 16384.0) / 1048576.0;     // (NVM - 2^14) / 2^20
    calibDataQ.par_p2  = ((double)calibData.par_p2  - 16384.0) / 536870912.0;   // (NVM - 2^14) / 2^29
    calibDataQ.par_p3  = (double)calibData.par_p3  / 4294967296.0;              // NVM / 2^32
    calibDataQ.par_p4  = (double)calibData.par_p4  / 137438953472.0;            // NVM / 2^37
    calibDataQ.par_p5  = (double)calibData.par_p5  / 0.125;                     // NVM / 2^-3 = NVM * 8
    calibDataQ.par_p6  = (double)calibData.par_p6  / 64.0;                      // NVM / 2^6
    calibDataQ.par_p7  = (double)calibData.par_p7  / 256.0;                     // NVM / 2^8
    calibDataQ.par_p8  = (double)calibData.par_p8  / 32768.0;                   // NVM / 2^15
    calibDataQ.par_p9  = (double)calibData.par_p9  / 281474976710656.0;         // NVM / 2^48
    calibDataQ.par_p10 = (double)calibData.par_p10 / 281474976710656.0;         // NVM / 2^48
    calibDataQ.par_p11 = (double)calibData.par_p11 / 36893488147419103232.0;    // NVM / 2^65
}

void Baro::configureSensor()
{
    uint8_t osr_config = OSR_P_X8 | OSR_T_X1;
    writeReg8(REG_OSR, osr_config);

    writeReg8(REG_ODR, ODR_50_HZ);

    uint8_t iir_config = IIR_FILTER_COEF_3;
    writeReg8(REG_CONF, iir_config);

    uint8_t pwr_config = PWR_MODE_NORMAL | PWE_PRESS_EN | PWE_TEMP_EN;
    writeReg8(REG_PWR_CTRL, pwr_config);
}

float Baro::compensateTemperature(uint32_t raw_temp)
{
    // Section 8.5 — uses pre-converted PAR_ values from calibQ
    double partial_data1 = (double)raw_temp - calibDataQ.par_t1;
    double partial_data2 = partial_data1 * calibDataQ.par_t2;
    t_lin = partial_data2 + (partial_data1 * partial_data1) * calibDataQ.par_t3;
    return (float)t_lin;
}

float Baro::compensatePressure(uint32_t rawPress)
{
    // Section 8.6 — uses pre-converted PAR_ values from calibQ
    double partial_data1 = calibDataQ.par_p6 * t_lin;
    double partial_data2 = calibDataQ.par_p7 * (t_lin * t_lin);
    double partial_data3 = calibDataQ.par_p8 * (t_lin * t_lin * t_lin);
    double partial_out1  = calibDataQ.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1        = calibDataQ.par_p2 * t_lin;
    partial_data2        = calibDataQ.par_p3 * (t_lin * t_lin);
    partial_data3        = calibDataQ.par_p4 * (t_lin * t_lin * t_lin);
    double partial_out2  = (double)rawPress * (calibDataQ.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1         = (double)rawPress * (double)rawPress;
    double partial_data2b = calibDataQ.par_p9 + calibDataQ.par_p10 * t_lin;
    double partial_data3b = partial_data1 * partial_data2b;
    double partial_data4  = partial_data3b + ((double)rawPress * (double)rawPress * (double)rawPress) * calibDataQ.par_p11;

    return (float)(partial_out1 + partial_out2 + partial_data4);
}

void Baro::readPressure(float &pressure)
{
    uint32_t rawPress, rawTemp;
    readReg24(REG_DATA_3, &rawTemp);
    readReg24(REG_DATA_0, &rawPress);
    compensateTemperature(rawTemp); // Update t_lin to be accessed later
    pressure = compensatePressure(rawPress);
}

void Baro::readTemperature(float &temperature)
{
    uint32_t rawTemp = 0;
    readReg24(REG_DATA_3, &rawTemp);
    temperature = compensateTemperature(rawTemp);
}

void Baro::getAltitude(float &altitude, float seaLevelPressure)
{
    float pressure;
    readPressure(pressure);

    // Convert pressure to altitude using barometric formula
    // altitude = 44330 * (1 - (P/P0)^(1/5.255))
    // P = pressure in Pa, P0 = sea level pressure in Pa
    float pressureRatio = (pressure / 100.0f) / seaLevelPressure; // Convert Pa to hPa
    altitude = 44330.0f * (1.0f - powf(pressureRatio, 0.19029f));
}

void Baro::readAll(float &pressure, float &temperature, float &altitude, float seaLevelPressure)
{
    uint32_t raw_press, raw_temp;

    // Read both raw values in one go (burst read more efficient)
    readReg24(REG_DATA_3, &raw_temp);
    readReg24(REG_DATA_0, &raw_press);

    // Compensate temperature
    temperature = compensateTemperature(raw_temp);

    // Compensate pressure
    pressure = compensatePressure(raw_press);

    // Calculate altitude
    float pressureRatio = (pressure / 100.0f) / seaLevelPressure;
    altitude = 44330.0f * (1.0f - powf(pressureRatio, 0.1903f));
}
