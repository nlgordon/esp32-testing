//
// Created by Nate Gordon on 9/22/18.
//

#include <algorithm>
#include <string.h>
#include "hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "pimpl_impl.h"

using namespace std;

class hal::Esp32HardwareContext {
    vector<shared_ptr<Esp32Pin>> pins;

public:
    Esp32HardwareContext() : pins {40} {};
    Pin getPin(uint8_t pin);
};


class hal::Esp32Pin {
    gpio_num_t pin;
public:
    explicit Esp32Pin(uint8_t pin);
    uint8_t getPin() const;
    gpio_num_t getHwPin() const;
};


// hal::EspHardwareContext
hal::Pin hal::Esp32HardwareContext::getPin(uint8_t pin) {
    if (!pins[pin]) {
        pins[pin].reset(new Esp32Pin(pin));
    }

    return hal::Pin { pins[pin] };
}


// hal::Esp32Pin
hal::Esp32Pin::Esp32Pin(uint8_t pin) {
    this->pin = static_cast<gpio_num_t>(pin);
}

uint8_t hal::Esp32Pin::getPin() const {
    return this->pin;
}

gpio_num_t hal::Esp32Pin::getHwPin() const {
    return this->pin;
}


// hal::HardwareContext
hal::HardwareContext::HardwareContext() = default;

hal::Pin hal::HardwareContext::getPin(uint8_t pin) {
    return m->getPin(pin);
}

hal::HardwareContext::~HardwareContext() = default;


// hal::Pin
hal::Pin::Pin(shared_ptr<Esp32Pin>& pin) : m {pin} { }

uint8_t hal::Pin::getPin() const {
    return m->getPin();
}

hal::Pin::~Pin() = default;


// hal::GPIOPin
//hal::GPIOPin hal::Pin::convertToGpio() {
//    return GPIOPin(m);
//}
//
//hal::Pin::~Pin() = default;
//
//
//// hal::GPIOPin
//hal::GPIOPin::GPIOPin(pimpl_shared<hal::Esp32Pin> pin) {
//
//}



//GPIOPin::GPIOPin(int pin) : pinObj(pin) {
GPIOPin::GPIOPin(int pin) {
    this->pin = static_cast<gpio_num_t>(pin);
    setupGpioHardware();
}

//GPIOPin::GPIOPin(Pin pinObj) : pinObj(pinObj) {
//    setupGpioHardware();
//}

void GPIOPin::setupGpioHardware() const {
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << this->pin);
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void GPIOPin::high() {
    gpio_set_level(this->pin, 1);
}

void GPIOPin::low() {
    gpio_set_level(this->pin, 0);
}

SPIBus::SPIBus() {
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

SPIDevice::SPIDevice(int pinChipSelect, SPIBus &bus, uint8_t command_bits) : bus(bus) {
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

unique_ptr<vector<uint8_t>> SPIDevice::transfer(const uint16_t cmd, const uint64_t addr, const vector<uint8_t> &tx) const {
    esp_err_t ret;
    unsigned int tx_bytes = tx.size();
    unique_ptr<vector<uint8_t>> rx(new vector<uint8_t>(tx_bytes));

    unique_ptr<vector<uint8_t>> tx_reversed(new vector<uint8_t>(tx_bytes));
    reverse_copy(tx.begin(), tx.end(), tx_reversed->begin());

    spi_transaction_t transaction = {
            .flags = 0,
            .cmd = cmd,
            .addr = addr,
            .length = 8 * tx_bytes,
            .rxlength = 8 * tx_bytes,
            .user = nullptr,
            { .tx_buffer = tx_reversed->data() },
            { .rx_buffer = rx->data() }
    };

    ret = spi_device_transmit(spi, &transaction);
    ESP_ERROR_CHECK(ret);

    reverse(rx->begin(), rx->end());

    return rx;
}

std::unique_ptr<std::vector<uint8_t>> SPIDevice::transfer(const std::vector<uint8_t> &tx) const {
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

std::unique_ptr<uint8_t[]> SPIDevice::transfer(const uint8_t tx[], size_t length) const {
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
