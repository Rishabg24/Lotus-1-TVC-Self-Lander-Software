#ifndef W25FLASH_H
#define W25FLASH_H
#include <Arduino.h>
#include <SPI.h>

class W25Flash
{

public:
    W25Flash(uint8_t csPin, SPIClass &spi = SPI1); // take bus at construction
    bool begin(); // init bus, verify chip responds
    bool eraseSector(uint32_t addr); // erase 4KB sector containing addr
    bool eraseChip(); // erase entire chip (~10s)
    bool writeByteArray(uint32_t addr, const uint8_t *data, uint32_t len); // write len bytes from data to addr, erasing sector first if needed
    bool readByteArray(uint32_t addr, uint8_t *buffer, uint32_t len); // read len bytes from addr into buffer

private: 
    SPIClass *_spi; // Pointer to spi bus (spi1 for this project since mcu connected to flash on that bus)
    uint8_t _cs; // Chip select pin
    bool isAlive(); // check JEDEC ID = EF 40 16 for W25Q32JVSSIQ
    void writeEnable(); // send WREN before any write/erase operation
    bool waitForReady(uint32_t timeout_ms = 5000); // wait until WIP bit in status register is cleared, or timeout
    void select(); // pull cs low to select the chip for SPI transactions
    void deselect(); // pull cs high to end SPI transaction
};


#endif