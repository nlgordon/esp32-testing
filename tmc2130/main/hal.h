//
// Created by Nate Gordon on 9/22/18.
//

#ifndef TMC_2130_HAL_H
#define TMC_2130_HAL_H

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <vector>
#include <memory>
#include "pimpl.h"

#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PARALLEL_LINES GPIO_NUM_16

// Completely re-work this to a context manager which is a factory of all HW resources
namespace hal {
class Pin;
class GPIOPin;
class Esp32HardwareContext;
class Esp32Pin;
class Esp32GPIOPin;

class HardwareContext {
    pimpl<Esp32HardwareContext> m;

public:
    HardwareContext();
    ~HardwareContext();
    Pin pin(uint8_t pin);
    GPIOPin gpioPin(Pin &pin);
    GPIOPin gpioPin(uint8_t pin);
};

class Pin {
    pimpl_shared<Esp32Pin> m;
    friend Esp32HardwareContext;

public:
    explicit Pin(std::shared_ptr<Esp32Pin>& pin);
    ~Pin();
    uint8_t getPinNum() const;
};

class GPIOPin {
    pimpl_shared<Esp32GPIOPin> m;

public:
    explicit GPIOPin(std::shared_ptr<Esp32GPIOPin>& pin);
    ~GPIOPin();
    void high();
    void low();
};
}


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
    std::unique_ptr<std::vector<uint8_t>> transfer(uint16_t cmd, uint64_t addr, const std::vector<uint8_t> &tx) const;
    std::unique_ptr<std::vector<uint8_t>> transfer(const std::vector<uint8_t> &tx) const;
    std::unique_ptr<uint8_t[]> transfer(const uint8_t tx[], size_t length) const;
};

void delayMilliseconds(int milliseconds);
void printVector(const std::vector<uint8_t> &data, const std::string &label);
void printArray(const uint8_t data[], size_t length, const std::string &label);

#endif //TMC_2130_HAL_H
