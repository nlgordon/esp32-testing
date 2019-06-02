
#ifndef TMC_2130_HAL_PIMPL_H
#define TMC_2130_HAL_PIMPL_H

#include <vector>
#include <memory>
#include "pimpl.h"

namespace hal_pimpl {

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

#endif //TMC_2130_HAL_PIMPL_H
