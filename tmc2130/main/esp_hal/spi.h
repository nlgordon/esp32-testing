
#ifndef TMC_2130_ESP_HAL_SPI_H
#define TMC_2130_ESP_HAL_SPI_H

#include "esp_hal.h"
#include "pin.h"
#include <driver/spi_master.h>

namespace esp_hal {
    class Esp32SPIBus : public hal::SPIBus {
        Esp32HardwareContext& ctx;
        spi_host_device_t bus_num;
        Esp32Pin* mosi;
        Esp32Pin* miso;
        Esp32Pin* clock;
    public:
        Esp32SPIBus(Esp32HardwareContext& ctx, spi_host_device_t bus_num);
        spi_host_device_t getBusNum();
        void foo() override {};
    };

    class Esp32SPIDevice : public hal::SPIDevice {
        Esp32HardwareContext& ctx;
        Esp32SPIBus* bus;
        Esp32Pin* chip_select;
        spi_device_handle_t spi;

    public:
        Esp32SPIDevice(Esp32HardwareContext& ctx, Esp32SPIBus* bus, Esp32Pin* chip_select);
        std::unique_ptr<std::vector<uint8_t>> transfer(const std::vector<uint8_t> &tx) const override;
    };

}

#endif //TMC_2130_ESP_HAL_SPI_H
