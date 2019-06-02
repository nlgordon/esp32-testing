
#include <algorithm>
#include <string.h>
#include "hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "pimpl_impl.h"

#define PARALLEL_LINES 16
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18

using namespace std;

// OLD STUFF
oldhal::SPIBus::SPIBus() {
    esp_err_t ret;
    spi_bus_config_t buscfg={
            .mosi_io_num = PIN_NUM_MOSI,
            .miso_io_num = PIN_NUM_MISO,
            .sclk_io_num = PIN_NUM_CLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = PARALLEL_LINES*320*2+8,
            .flags = 0
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);
}

oldhal::SPIDevice::SPIDevice(int pinChipSelect, SPIBus &bus, uint8_t command_bits) : bus(bus) {
    esp_err_t ret;
    spi_device_interface_config_t devcfg={
            .command_bits = command_bits,
            .address_bits = 0,
            .dummy_bits = 0,
            .mode = 3,
            .duty_cycle_pos = 0,
            .cs_ena_pretrans = 0,
            .cs_ena_posttrans = 0,
            .clock_speed_hz = 1*1000*1000,           //Clock out at 1 MHz
            .input_delay_ns = 0,
            .spics_io_num = pinChipSelect,               //CS pin
            .flags = 0,
            .queue_size = 7,                          //We want to be able to queue 7 transactions at a time
            .pre_cb = nullptr,
            .post_cb = nullptr
    };
    printf("Initialized and adding device\n");
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

std::unique_ptr<std::vector<uint8_t>> oldhal::SPIDevice::transfer(const std::vector<uint8_t> &tx) {
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

std::unique_ptr<uint8_t[]> oldhal::SPIDevice::transfer(const uint8_t tx[], size_t length) const {
    esp_err_t ret;
    auto rx_data = new uint8_t[length];
    unique_ptr<uint8_t[]> rx{rx_data};

    memset(rx_data, 0, length);

    spi_transaction_t transaction = {
            .flags = 0,
            .cmd = 0,
            .addr = 0,
            .length = 8 * length,
            .rxlength = 8 * length,
            .user = nullptr,
            { .tx_buffer = tx },
            { .rx_buffer = rx_data }
    };

    ret = spi_device_transmit(spi, &transaction);
    ESP_ERROR_CHECK(ret);

    return rx;
}

void printVector(const vector <uint8_t> &data, const std::string &label) {
    printf("%s : ", label.c_str());
    for (auto item : data) {
        printf("0x%02x ", item);
    }
    printf("\n");
}

void delayMilliseconds(int milliseconds) {
    vTaskDelay(milliseconds / portTICK_RATE_MS);
}
