#include "flightLogger.h"


// Static constexpr members are defined in the header, so no definitions
// needed here. Just initialize the instance variables.
FlightLogger::FlightLogger()
    : flash(10, SPI1), _nextAddr(DATA_START), _packetCount(0) // use SPI1 for flash to avoid conflicts with IMU on SPI
{
}


bool FlightLogger::begin()
{
    // SPI1.begin(); // use SPI1 for flash to avoid conflicts with IMU on SPI
    if (!flash.begin())
    {
        // Serial.println("[FlightLogger] ERROR: Flash chip not found.");
        return false;
    }
    loadOrInitHeader();
    // Serial.printf("[FlightLogger] Ready. %u packets on flash.\n", _packetCount);
    return true;
}

bool FlightLogger::logPacket(const FlightPacket &packet)
{
    // Bail out if flash is full
    if (_nextAddr + sizeof(FlightPacket) >= FLASH_SIZE)
    {
        Serial.println("[FlightLogger] ERROR: Flash full.");
        return false;
    }

    // Work on a local copy so we can fill in the checksum
    FlightPacket packetCopy = packet;
    packetCopy.checksum = xorChecksum((uint8_t *)&packetCopy, sizeof(packetCopy) - 1);

    // If we've crossed into a new 4 KB sector, erase it first.
    // NOR flash can only write 1→0; erasing resets a whole sector to 0xFF.
    if (_nextAddr % 4096 == 0)
    {
        flash.eraseSector(_nextAddr);
    }

    flash.writeByteArray(_nextAddr, (uint8_t *)&packetCopy, sizeof(packetCopy));
    _nextAddr    += sizeof(FlightPacket);
    _packetCount += 1;

    // Save header every 50 packets so we don't lose too much on power loss.
    // Writing every packet would wear out the header sector quickly.
    if (_packetCount % 50 == 0)
    {
        saveHeader();
    }

    return true;
}

void FlightLogger::dump()
{
    // Print CSV header so your Python script knows the column order
    Serial.println("timestamp_ms,altitude_m,velocity_ms,"
                   "accel_x,accel_y,accel_z,"
                   "gyro_x,gyro_y,gyro_z,"
                   "servoX,servoY,state,checksum_ok");

    uint32_t addr = DATA_START;

    for (uint32_t i = 0; i < _packetCount; i++)
    {
        FlightPacket pkt;
        flash.readByteArray(addr, (uint8_t *)&pkt, sizeof(pkt));

        // Verify integrity: recompute checksum over the data bytes
        uint8_t expected = xorChecksum((uint8_t *)&pkt, sizeof(pkt) - 1);
        bool    ok       = (pkt.checksum == expected);

        Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%s\n",
                      pkt.timestamp_ms,
                      pkt.altitude_m,
                      pkt.velocity_ms,
                      pkt.accel_x, pkt.accel_y, pkt.accel_z,
                      pkt.gyro_x,  pkt.gyro_y,  pkt.gyro_z,
                      pkt.servoX,  pkt.servoY,
                      pkt.state,
                      ok ? "OK" : "BAD");

        addr += sizeof(FlightPacket);
    }

    Serial.printf("[FlightLogger] Dump complete. %u packets.\n", _packetCount);
}

void FlightLogger::eraseAll()
{
    Serial.println("[FlightLogger] Erasing chip... (~10s)");
    flash.eraseChip();
    initHeader();
    Serial.println("[FlightLogger] Erase complete.");
}


uint8_t FlightLogger::xorChecksum(uint8_t *data, size_t len)
{
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++)
    {
        c ^= data[i];
    }
    return c;
}

void FlightLogger::loadOrInitHeader()
{
    FlashHeader h;
    flash.readByteArray(HEADER_ADDR, (uint8_t *)&h, sizeof(h));

    if (h.magic == MAGIC)
    {
        // Valid header found — pick up where we left off
        _nextAddr    = h.nextWriteAddr;
        _packetCount = h.packetCount;
    }
    else
    {
        // No valid header — fresh chip or first boot
        initHeader();
    }
}

void FlightLogger::saveHeader()
{
    FlashHeader h;
    h.magic         = MAGIC;
    h.packetCount   = _packetCount;
    h.nextWriteAddr = _nextAddr;
    h.armed         = 1;

    // Header lives in sector 0 — must erase before rewriting
    flash.eraseSector(HEADER_ADDR);
    flash.writeByteArray(HEADER_ADDR, (uint8_t *)&h, sizeof(h));
}

void FlightLogger::initHeader()
{
    _nextAddr    = DATA_START;
    _packetCount = 0;
    saveHeader();
}