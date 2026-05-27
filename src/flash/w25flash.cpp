#include "w25flash.h"
#include <SPI.h>
#include <Arduino.h>

W25Flash::W25Flash(uint8_t csPin, SPIClass &spi) : _cs(csPin), _spi(&spi)
{
}

void W25Flash::select()
{
    digitalWrite(_cs, LOW);
}

void W25Flash::deselect()
{
    digitalWrite(_cs, HIGH);
}

bool W25Flash::begin()
{
    pinMode(_cs, OUTPUT);
    deselect(); // ensure chip is not selected
    _spi->begin();
    return isAlive();
}

void W25Flash::writeEnable()
{
    select();
    _spi->transfer(0x06);
    deselect();
}

bool W25Flash::waitForReady(uint32_t timeout_ms)
{
    select();
    _spi->transfer(0x05);
    uint32_t time_start = millis();
    while (1)
    {
        if (millis() - time_start >= timeout_ms)
        {
            deselect();
            return false;
        }

        uint8_t status = _spi->transfer(0x00);

        if (status & 0x01)
        { // bit 0 is set meaning still busy
            continue;
        }
        else
        {
            deselect();
            return true;
        }
    }
}

bool W25Flash::isAlive()
{
    select();
    _spi->transfer(0x9F); // JEDEC ID command

    uint8_t b1 = _spi->transfer(0x00); // manufacturer ID
    uint8_t b2 = _spi->transfer(0x00); // memory type
    uint8_t b3 = _spi->transfer(0x00); // capacity
    deselect();
    return (b1 == 0xEF && b2 == 0x40 && b3 == 0x16);
}

bool W25Flash::eraseSector(uint32_t addr)
{
    writeEnable();
    select();
    _spi->transfer(0x20);                // SECTOR ERASE command
    _spi->transfer((addr >> 16) & 0xFF); // 24-bit address
    _spi->transfer((addr >> 8) & 0xFF);
    _spi->transfer(addr & 0xFF);
    deselect();
    return waitForReady();
}

bool W25Flash::eraseChip()
{
    writeEnable();
    select();
    _spi->transfer(0xC7);
    deselect();
    return waitForReady(15000); // chip erase can take up to 15s
}

bool W25Flash::writeByteArray(uint32_t addr, const uint8_t *data, uint32_t len)
{
    // W25Q32 can only write 256 bytes at a time, and cannot write across page boundaries
    // This function will handle splitting the data into chunks and erasing sectors as needed
    uint32_t remaining = len;
    uint32_t offset = 0; // how far into data[] we are

    while (remaining > 0)
    {
        // how many bytes are left in the current 256-byte page?
        uint32_t pageSpace = 256 - (addr & 0xFF);

        // don't write more than what's remaining
        uint32_t toWrite = min(pageSpace, remaining);

        writeEnable();
        select();

        _spi->transfer(0x02); // Page Program Command

        _spi->transfer((addr >> 16) & 0XFF);
        _spi->transfer((addr >> 8) & 0xFF);
        _spi->transfer(addr & 0xFF);

        for (uint32_t i = 0; i < toWrite; i++)
        {
            _spi->transfer(data[offset + i]);
        }
        deselect();
        if (!waitForReady())
            return false;

        // advance for next iteration
        addr += toWrite;
        offset += toWrite;
        remaining -= toWrite;
    }
    return true;
}

bool W25Flash::readByteArray(uint32_t addr, uint8_t *buffer, uint32_t len)
{

    select();
    _spi->transfer(0x03);
    _spi->transfer((addr >> 16) & 0XFF);
    _spi->transfer((addr >> 8) & 0xFF);
    _spi->transfer(addr & 0xFF);

    for (uint32_t i = 0; i < len; i++)
    {
        buffer[i] = _spi->transfer(0x00);
    }

    deselect();

    return true;
}
