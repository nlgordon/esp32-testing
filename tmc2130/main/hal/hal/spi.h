
#ifndef TMC_2130_SPI_H
#define TMC_2130_SPI_H

#include <vector>
#include <memory>

namespace hal {
    typedef enum {
        BUS_1=1,
        BUS_2=2
    } spi_bus_num;

    class SPIBus {
        // TODO: This is just to make this class polymorphic
        virtual void foo() = 0;
    };

    class SPIDevice {
    public:
        virtual std::unique_ptr<std::vector<uint8_t>> transfer(const std::vector<uint8_t> &tx) const = 0;
    };
}

#endif //TMC_2130_SPI_H
