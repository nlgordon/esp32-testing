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

// Completely re-work this to a context manager which is a factory of all HW resources
namespace hal {

    typedef enum {
        BUS_1=1,
        BUS_2=2
    } spi_bus_num;
class Pin;
class GPIOPin;
class SPIBus;
class SPIDevice;
class Esp32HardwareContext;
class Esp32Pin;
class Esp32GPIOPin;
class Esp32SPIBus;
class Esp32SPIDevice;

class HardwareContext {
    pimpl<Esp32HardwareContext> m;

public:
    HardwareContext();
    ~HardwareContext();
    Pin pin(uint8_t pin);
    GPIOPin gpioPin(Pin &pin);
    GPIOPin gpioPin(uint8_t pin);
    SPIBus spiBus(Pin &mosi, Pin &miso, Pin &clock);
    SPIBus spiBus(spi_bus_num bus);
    SPIDevice spiDevice(SPIBus &bus, Pin &chip_select);
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

class SPIBus {
    pimpl_shared<Esp32SPIBus> m;
    friend Esp32HardwareContext;

public:
    explicit SPIBus(std::shared_ptr<Esp32SPIBus>& bus);
    ~SPIBus();
};

class SPIDevice {
    pimpl_shared<Esp32SPIDevice> m;

public:
    explicit SPIDevice(std::shared_ptr<Esp32SPIDevice>& device);
    ~SPIDevice();

    std::unique_ptr<std::vector<uint8_t>> transfer(const std::vector<uint8_t> &tx) const;
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
    std::unique_ptr<std::vector<uint8_t>> transfer(const std::vector<uint8_t> &tx);
};

void delayMilliseconds(int milliseconds);
void printVector(const std::vector<uint8_t> &data, const std::string &label);
void printArray(const uint8_t data[], size_t length, const std::string &label);

#endif //TMC_2130_HAL_H
