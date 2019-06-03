#ifndef TMC_2130_HAL_H
#define TMC_2130_HAL_H

#include <vector>
#include <memory>

#include "pin.h"
#include "gpiopin.h"
#include "spi.h"

namespace hal {

    class HardwareContext {
    public:
        virtual Pin* pin(uint8_t pin) = 0;
        virtual GPIOPin* gpioPin(Pin* pin) = 0;
        virtual GPIOPin* gpioPin(uint8_t pin) = 0;
        virtual SPIBus* spiBus(spi_bus_num bus) = 0;
        virtual SPIDevice* spiDevice(SPIBus* bus, Pin* chip_select) = 0;
    };

    class ContextFactory {
    public:
        virtual std::unique_ptr <HardwareContext> createContext();
    };
}

#endif //TMC_2130_HAL_H
