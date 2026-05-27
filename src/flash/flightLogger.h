#ifndef FLIGHTLOGGER_H
#define FLIGHTLOGGER_H

#include <Arduino.h>
#include "w25flash.h"

// ─────────────────────────────────────────────
//  Data Structs
// ─────────────────────────────────────────────

// One packet of flight data logged per loop tick.
// __attribute__((packed)) removes any padding bytes so the struct
// is exactly the same size in memory as on flash — critical for parsing.
struct __attribute__((packed)) FlightPacket
{
    uint32_t timestamp_ms;          // millis() at time of logging
    float    altitude_m;            // barometer altitude in metres
    float    velocity_ms;           // estimated vertical velocity m/s
    float    accel_x, accel_y, accel_z; // IMU accelerometer (m/s²)
    float    gyro_x,  gyro_y,  gyro_z;  // IMU gyroscope (deg/s)
    float    servoX, servoY;        // TVC servo commanded positions (deg)
    uint8_t  state;                 // flight state machine value
    uint8_t  checksum;              // XOR of all bytes before this field
};

// Written to the very first sector of flash.
// Lets the firmware know whether valid data exists on boot.
struct __attribute__((packed)) FlashHeader
{
    uint32_t magic;          // 0xDEADBEEF if header is valid
    uint32_t packetCount;    // total packets written
    uint32_t nextWriteAddr;  // where the next packet will go
    uint32_t armed;          // 1 if rocket was armed before this session
};

// ─────────────────────────────────────────────
//  FlightLogger Class
// ─────────────────────────────────────────────

class FlightLogger
{
public:
    FlightLogger();

    // Call in setup(). Returns false if the flash chip doesn't respond.
    bool begin();

    // Write one FlightPacket to flash. Returns false if flash is full.
    bool logPacket(const FlightPacket &packet);

    // Stream every stored packet out over Serial as CSV.
    // Call this on the ground to pull data off the rocket.
    void dump();

    // Erase the entire chip. Takes ~10 seconds.
    // Always call this before a flight.
    void eraseAll();

    // How many packets have been written this session.
    uint32_t packetCount() const { return _packetCount;}

private:
    W25Flash flash{10, SPI1}; // use SPI1 for flash to avoid conflicts with IMU on SPI
    uint32_t _nextAddr;
    uint32_t _packetCount;

    // Flash layout constants
    static constexpr uint32_t HEADER_ADDR = 0x000000; // sector 0: bookkeeping
    static constexpr uint32_t DATA_START  = 0x001000; // sector 1 onward: packets
    static constexpr uint32_t FLASH_SIZE  = 0x400000; // 4 MB (W25Q32)
    static constexpr uint32_t MAGIC       = 0xDEADBEEF;

    // Compute XOR checksum over raw bytes
    uint8_t xorChecksum(uint8_t *data, size_t len);

    void loadOrInitHeader(); // read header on boot; init if not found
    void saveHeader();       // write current state back to header sector
    void initHeader();       // fresh start: reset addresses, write blank header
};

#endif // FLIGHTLOGGER_H