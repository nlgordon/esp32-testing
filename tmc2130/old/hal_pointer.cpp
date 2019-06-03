#include "hal_pointer.h"
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <string.h>

#define PARALLEL_LINES 16

using namespace std;
using namespace hal_pointer;

class hal_pointer::Esp32Pin : public Pin {
    gpio_num_t pin;
    HardwareContext& ctx;
public:
    explicit Esp32Pin(HardwareContext &ctx, uint8_t pin);
    uint8_t getPinNum() const override;
    gpio_num_t getHwPinNum() const;
};

class hal_pointer::Esp32GPIOPin : public GPIOPin {
    Esp32Pin* pin;
    void setupGpioHardware() const;
    HardwareContext& ctx;

public:
    explicit Esp32GPIOPin(HardwareContext &ctx, Esp32Pin *pin);
    void high() override;
    void low() override;
};

class hal_pointer::Esp32SPIBus : public SPIBus {
    HardwareContext& ctx;
    spi_host_device_t bus_num;
    Esp32Pin* mosi;
    Esp32Pin* miso;
    Esp32Pin* clock;
public:
    Esp32SPIBus(HardwareContext& ctx, spi_host_device_t bus_num);
    spi_host_device_t getBusNum();
};

class hal_pointer::Esp32SPIDevice : public SPIDevice {
    HardwareContext& ctx;
    Esp32SPIBus* bus;
    Esp32Pin* chip_select;
    spi_device_handle_t spi;

public:
    Esp32SPIDevice(HardwareContext& ctx, Esp32SPIBus* bus, Esp32Pin* chip_select);
    unique_ptr<vector<uint8_t>> transfer(const vector<uint8_t> &tx) const override;
};

hal_pointer::HardwareContext::HardwareContext() : pins{40}, gpioPins{40}, spiBuses{3}, spiDevices{40} {

}

hal_pointer::Pin *hal_pointer::HardwareContext::pin(uint8_t pin) {
    if (!pins[pin]) {
        pins[pin] = std::make_unique<Esp32Pin>(this, pin);
    }

    return pins[pin].get();
}

GPIOPin *HardwareContext::gpioPin(Pin *pin) {
    auto espPin = dynamic_cast<Esp32Pin*>(pin);
    uint8_t pin_num = espPin->getPinNum();

    if (!gpioPins[pin_num]) {
        gpioPins[pin_num] = std::make_unique<Esp32GPIOPin>(this, espPin);
    }
    return gpioPins[pin_num].get();
}

GPIOPin *HardwareContext::gpioPin(uint8_t pin) {
    return gpioPin(this->pin(pin));
}

SPIBus *HardwareContext::spiBus(spi_bus_num bus) {
    spi_host_device_t hw_bus = bus == BUS_1 ? HSPI_HOST : VSPI_HOST;

    if (!spiBuses[hw_bus]) {
        spiBuses[hw_bus] = std::make_unique<Esp32SPIBus>(this, hw_bus);
    }
    return spiBuses[hw_bus].get();
}

SPIDevice *HardwareContext::spiDevice(SPIBus *bus, Pin *chip_select) {
    uint8_t chip_select_pin = chip_select->getPinNum();

    if (!spiDevices[chip_select_pin]) {
        spiDevices[chip_select_pin] = std::make_unique<Esp32SPIDevice>(this, bus, chip_select);
    }

    return spiDevices[chip_select_pin].get();
}

hal_pointer::HardwareContext::~HardwareContext() = default;

// hal::Esp32Pin
hal_pointer::Esp32Pin::Esp32Pin(HardwareContext &ctx, uint8_t pin) : ctx{ctx} {
    this->pin = static_cast<gpio_num_t>(pin);
}

uint8_t hal_pointer::Esp32Pin::getPinNum() const {
    return this->pin;
}

gpio_num_t hal_pointer::Esp32Pin::getHwPinNum() const {
    return this->pin;
}

// hal::Esp32GPIOPin
hal_pointer::Esp32GPIOPin::Esp32GPIOPin(HardwareContext &ctx, Esp32Pin *pin) : pin {pin}, ctx{ctx} {
    setupGpioHardware();
}

void hal_pointer::Esp32GPIOPin::setupGpioHardware() const {
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << this->pin->getHwPinNum());
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void hal_pointer::Esp32GPIOPin::high() {
    gpio_set_level(this->pin->getHwPinNum(), 1);
}

void hal_pointer::Esp32GPIOPin::low() {
    gpio_set_level(this->pin->getHwPinNum(), 0);
}


// hal::Esp32SPIBus
hal_pointer::Esp32SPIBus::Esp32SPIBus(HardwareContext& ctx, spi_host_device_t bus_num) : bus_num{bus_num}, ctx{ctx} {
    gpio_num_t mosi_pin = bus_num == HSPI_HOST ? GPIO_NUM_13 : GPIO_NUM_23;
    gpio_num_t miso_pin = bus_num == HSPI_HOST ? GPIO_NUM_12 : GPIO_NUM_19;
    gpio_num_t clock_pin = bus_num == HSPI_HOST ? GPIO_NUM_14 : GPIO_NUM_18;

    mosi = dynamic_cast<Esp32Pin*>(ctx.pin(mosi_pin));
    miso = dynamic_cast<Esp32Pin*>(ctx.pin(miso_pin));
    clock = dynamic_cast<Esp32Pin*>(ctx.pin(clock_pin));

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

spi_host_device_t hal_pointer::Esp32SPIBus::getBusNum() {
    return bus_num;
}

// hal::Esp32SPIDevice
hal_pointer::Esp32SPIDevice::Esp32SPIDevice(HardwareContext& ctx, Esp32SPIBus* bus, Esp32Pin* chip_select) : ctx{ctx}, bus { bus }, chip_select { chip_select }, spi{nullptr} {
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

unique_ptr<vector<uint8_t>> hal_pointer::Esp32SPIDevice::transfer(const vector<uint8_t> &tx) const {
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
