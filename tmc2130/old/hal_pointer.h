#ifndef TMC_2130_HAL_POINTER_H
#define TMC_2130_HAL_POINTER_H

#include <stdint.h>
#include <vector>
#include <memory>

namespace hal_pointer {
    class Pin {
    public:
        virtual uint8_t getPinNum() const = 0;
    };

    class GPIOPin {
    public:
        virtual void high() = 0;
        virtual void low() = 0;
    };

    typedef enum {
        BUS_1=1,
        BUS_2=2
    } spi_bus_num;

    class SPIBus {

    };

    class SPIDevice {
    public:
        virtual std::unique_ptr<std::vector<uint8_t>> transfer(const std::vector<uint8_t> &tx) const = 0;
    };

    class HardwareContext {
        std::vector<std::unique_ptr<Pin>> pins;
        std::vector<std::unique_ptr<GPIOPin>> gpioPins;
        std::vector<std::unique_ptr<SPIBus>> spiBuses;
        std::vector<std::unique_ptr<SPIDevice>> spiDevices;
    public:
        HardwareContext();
        ~HardwareContext();
        Pin* pin(uint8_t pin);
        GPIOPin* gpioPin(Pin* pin);
        GPIOPin* gpioPin(uint8_t pin);
        SPIBus* spiBus(spi_bus_num bus);
        SPIDevice* spiDevice(SPIBus* bus, Pin* chip_select);
    };

    class Esp32Pin;
    class Esp32GPIOPin;
    class Esp32SPIBus;
    class Esp32SPIDevice;
}

#endif //TMC_2130_HAL_POINTER_H
