//
// Created by Nate Gordon on 9/22/18.
//

#ifndef TMC_2130_HAL_H
#define TMC_2130_HAL_H

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <vector>
#include <memory>

#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PARALLEL_LINES GPIO_NUM_16

class Pin {
    gpio_num_t pin;

public:
    Pin(int pin);
    gpio_num_t getPin();
};

class GPIOPin {
    gpio_num_t pin;
    Pin pinObj = 0;
    void setupGpioHardware() const;
public:
    GPIOPin(int pin);
    GPIOPin(Pin pinObj);

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
    std::unique_ptr<std::vector<uint8_t>> transfer(const std::vector<uint8_t> &tx);
};

void delayMilliseconds(int milliseconds);
void printVector(const std::vector<uint8_t> &data, const std::string &label);

#endif //TMC_2130_HAL_H
