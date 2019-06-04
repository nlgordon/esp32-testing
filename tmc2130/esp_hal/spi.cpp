
#include "spi.h"
#include <memory>
#include <string.h>
#define PARALLEL_LINES 16

using namespace esp_hal;
using namespace std;

// hal::Esp32SPIBus
Esp32SPIBus::Esp32SPIBus(Esp32HardwareContext* ctx, spi_host_device_t bus_num) : bus_num{bus_num}, ctx{ctx} {
    gpio_num_t mosi_pin = bus_num == HSPI_HOST ? GPIO_NUM_13 : GPIO_NUM_23;
    gpio_num_t miso_pin = bus_num == HSPI_HOST ? GPIO_NUM_12 : GPIO_NUM_19;
    gpio_num_t clock_pin = bus_num == HSPI_HOST ? GPIO_NUM_14 : GPIO_NUM_18;

    mosi = dynamic_cast<Esp32Pin*>(ctx->pin(mosi_pin));
    miso = dynamic_cast<Esp32Pin*>(ctx->pin(miso_pin));
    clock = dynamic_cast<Esp32Pin*>(ctx->pin(clock_pin));

    esp_err_t ret;
    spi_bus_config_t buscfg={
            .mosi_io_num = mosi->getHwPinNum(),
            .miso_io_num = miso->getHwPinNum(),
            .sclk_io_num = clock->getHwPinNum(),
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = PARALLEL_LINES*320*2+8,
            .flags = 0
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(bus_num, &buscfg, 0);
    ESP_ERROR_CHECK(ret);
}

spi_host_device_t Esp32SPIBus::getBusNum() {
    return bus_num;
}

hal::SPIDevice *Esp32SPIBus::device(hal::Pin *chip_select) {
    return ctx->spiDevice(this, chip_select);
}

hal::SPIDevice *Esp32SPIBus::device(uint8_t chip_select) {
    return device(ctx->pin(chip_select));
}

// hal::Esp32SPIDevice
Esp32SPIDevice::Esp32SPIDevice(Esp32HardwareContext* ctx, Esp32SPIBus* bus, Esp32Pin* chip_select) : ctx{ctx}, bus { bus }, chip_select { chip_select }, spi{nullptr} {
    esp_err_t ret;
    spi_device_interface_config_t devcfg={
            .command_bits = 0,
            .address_bits = 0,
            .dummy_bits = 0,
            .mode = 3,
            .duty_cycle_pos = 0,
            .cs_ena_pretrans = 0,
            .cs_ena_posttrans = 0,
            .clock_speed_hz = 1*1000*1000,           //Clock out at 1 MHz
            .input_delay_ns = 0,
            .spics_io_num = chip_select->getHwPinNum(),               //CS pin
            .flags = 0,
            .queue_size = 7,                          //We want to be able to queue 7 transactions at a time
            .pre_cb = nullptr,
            .post_cb = nullptr
    };
    ret=spi_bus_add_device(bus->getBusNum(), &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

unique_ptr<vector<uint8_t>> Esp32SPIDevice::transfer(const vector<uint8_t> &tx) const {
    esp_err_t ret;
    unsigned int tx_bytes = tx.size();
    unique_ptr<vector<uint8_t>> rx(new vector<uint8_t>(tx_bytes));

    memset(rx->data(), 1, tx_bytes);

    spi_transaction_t transaction = {
            .flags = 0,
            .cmd = 0,
            .addr = 0,
            .length = 8 * tx_bytes,
            .rxlength = 8 * tx_bytes,
            .user = nullptr,
            { .tx_buffer = tx.data() },
            { .rx_buffer = rx->data() }
    };

    ret = spi_device_transmit(spi, &transaction);
    ESP_ERROR_CHECK(ret);

    return rx;
}
