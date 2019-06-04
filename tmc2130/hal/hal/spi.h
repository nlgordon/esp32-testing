
#ifndef TMC_2130_SPI_H
#define TMC_2130_SPI_H

#include <vector>
#include <memory>
#include "pin.h"

namespace hal {
    enum class SpiBusNum {
        BUS_1=1,
        BUS_2=2
    };

    class SPIDevice {
    public:
        virtual std::unique_ptr<std::vector<uint8_t>> transfer(const std::vector<uint8_t> &tx) const = 0;
    };

    class SPIBus {
    public:
        virtual SPIDevice* device(Pin* chip_select) = 0;
        virtual SPIDevice* device(uint8_t chip_select) = 0;
    };
}

#endif //TMC_2130_SPI_H
