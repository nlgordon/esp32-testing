//
// Created by Nate Gordon on 9/22/18.
//

#ifndef TMC_2130_HAL_H
#define TMC_2130_HAL_H

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <vector>
#include <memory>

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PARALLEL_LINES 16

class GPIOPin {
private:
    gpio_num_t pin;
public:
    GPIOPin(int pin);

    void high();

    void low();
};


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
    std::unique_ptr<std::vector<uint8_t>> transfer(uint16_t cmd, uint64_t addr, const std::vector<uint8_t> &tx);
    std::unique_ptr<std::vector<uint8_t>> transfer(const std::vector<uint8_t> &tx);
};

void delayMilliseconds(int milliseconds);
void printVector(const std::vector<uint8_t> &data);

#endif //TMC_2130_HAL_H
