#ifndef TMC_2130_ESP_HAL_H
#define TMC_2130_ESP_HAL_H

#include <hal/hal.h>

namespace esp_hal {
class Esp32HardwareContext : public hal::HardwareContext {
    std::vector<std::unique_ptr<hal::Pin>> pins;
    std::vector<std::unique_ptr<hal::GPIOPin>> gpioPins;
    std::vector<std::unique_ptr<hal::SPIBus>> spiBuses;
    std::vector<std::unique_ptr<hal::SPIDevice>> spiDevices;
public:
    Esp32HardwareContext();
    ~Esp32HardwareContext();
    hal::Pin* pin(uint8_t pin) override;
    hal::GPIOPin* gpioPin(hal::Pin* pin) override;
    hal::GPIOPin* gpioPin(uint8_t pin) override;
    hal::SPIBus* spiBus(hal::spi_bus_num bus) override;
    hal::SPIDevice* spiDevice(hal::SPIBus* bus, hal::Pin* chip_select) override;
};
}

#endif //TMC_2130_ESP_HAL_H
