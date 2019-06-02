
#ifndef TMC_2130_HAL_H
#define TMC_2130_HAL_H

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <vector>
#include <memory>
#include "pimpl.h"

// Completely re-work this to a context manager which is a factory of all HW resources
namespace oldhal {

class SPIBus {
private:

public:
    SPIBus();
};

class SPIDevice {
private:
    spi_device_handle_t spi;
    SPIBus &bus;
public:
    SPIDevice(int pinChipSelect, SPIBus &bus, uint8_t command_bits);
    std::unique_ptr<std::vector<uint8_t>> transfer(const std::vector<uint8_t> &tx);
};

}

void delayMilliseconds(int milliseconds);
void printVector(const std::vector<uint8_t> &data, const std::string &label);
void printArray(const uint8_t data[], size_t length, const std::string &label);

#endif //TMC_2130_HAL_H
