
#include "hal_pimpl.h"

#include <algorithm>
#include <string.h>
#include "hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "pimpl_impl.h"
#include <driver/gpio.h>
#include <driver/spi_master.h>

#define PARALLEL_LINES 16

using namespace std;

class hal_pimpl::Esp32HardwareContext {
    vector<shared_ptr<Esp32Pin>> pins;
    vector<shared_ptr<Esp32GPIOPin>> gpioPins;
    vector<shared_ptr<Esp32SPIBus>> spiBuses;
    vector<shared_ptr<Esp32SPIDevice>> spiDevices;

public:
    Esp32HardwareContext() : pins {40}, gpioPins {40}, spiBuses {3}, spiDevices {40} {};
    Pin pin(uint8_t pin);
    GPIOPin gpioPin(const Pin &pin);
    GPIOPin gpioPin(uint8_t pin);
    SPIBus spiBus(Pin &mosi, Pin &miso, Pin &clock);
    SPIBus spiBus(spi_bus_num bus);
    SPIDevice spiDevice(SPIBus &bus, Pin &chip_select);
};


class hal_pimpl::Esp32Pin {
    gpio_num_t pin;
public:
    explicit Esp32Pin(uint8_t pin);
    uint8_t getPinNum() const;
    gpio_num_t getHwPinNum() const;
};

class hal_pimpl::Esp32GPIOPin {
    shared_ptr<hal_pimpl::Esp32Pin> pin;
    void setupGpioHardware() const;

public:
    explicit Esp32GPIOPin(shared_ptr<Esp32Pin>& pin);
    explicit Esp32GPIOPin(shared_ptr<Esp32Pin>&& pin);
    void high();
    void low();
};

class hal_pimpl::Esp32SPIBus {
    spi_host_device_t bus_num;
    shared_ptr<hal_pimpl::Esp32Pin> mosi;
    shared_ptr<hal_pimpl::Esp32Pin> miso;
    shared_ptr<hal_pimpl::Esp32Pin> clock;
public:
    Esp32SPIBus(shared_ptr<Esp32Pin> &&mosi, shared_ptr<Esp32Pin> &&miso, shared_ptr<Esp32Pin> &&clock);
    spi_host_device_t getBusNum();
};

class hal_pimpl::Esp32SPIDevice {
    shared_ptr<Esp32SPIBus> bus;
    shared_ptr<Esp32Pin> chip_select;

public:
    Esp32SPIDevice(shared_ptr<Esp32SPIBus> &&bus, shared_ptr<Esp32Pin> &&chip_select);
};


// hal::EspHardwareContext
hal_pimpl::Pin hal_pimpl::Esp32HardwareContext::pin(uint8_t pin) {
    if (!pins[pin]) {
        pins[pin].reset(new Esp32Pin(pin));
    }

    return hal_pimpl::Pin { pins[pin] };
}

hal_pimpl::GPIOPin hal_pimpl::Esp32HardwareContext::gpioPin(const Pin &pin) {
    uint8_t pinNum { pin.m->getPinNum() };
    if (!gpioPins[pinNum]) {
        gpioPins[pinNum].reset(new Esp32GPIOPin(pin.m.share()));
    }
    return hal_pimpl::GPIOPin { gpioPins[pinNum] };
}

hal_pimpl::GPIOPin hal_pimpl::Esp32HardwareContext::gpioPin(uint8_t pin) {
    return gpioPin(this->pin(pin));
}

hal_pimpl::SPIBus hal_pimpl::Esp32HardwareContext::spiBus(hal_pimpl::Pin &mosi, hal_pimpl::Pin &miso, hal_pimpl::Pin &clock) {
    spiBuses[2].reset(new Esp32SPIBus(mosi.m.share(), miso.m.share(), clock.m.share()));
    return hal_pimpl::SPIBus {spiBuses[0] };
}

hal_pimpl::SPIBus hal_pimpl::Esp32HardwareContext::spiBus(spi_bus_num bus) {
    spi_host_device_t hw_bus = bus == BUS_1 ? HSPI_HOST : VSPI_HOST;

    gpio_num_t mosi_pin = hw_bus == HSPI_HOST ? GPIO_NUM_13 : GPIO_NUM_23;
    gpio_num_t miso_pin = hw_bus == HSPI_HOST ? GPIO_NUM_12 : GPIO_NUM_19;
    gpio_num_t clock_pin = hw_bus == HSPI_HOST ? GPIO_NUM_14 : GPIO_NUM_18;

    spiBuses[hw_bus].reset(new Esp32SPIBus(pin(mosi_pin).m.share(), pin(miso_pin).m.share(), pin(clock_pin).m.share()));
    return hal_pimpl::SPIBus { spiBuses[hw_bus] };
}

hal_pimpl::SPIDevice hal_pimpl::Esp32HardwareContext::spiDevice(hal_pimpl::SPIBus &bus, hal_pimpl::Pin &chip_select) {
    uint8_t chip_select_pin_num = chip_select.m->getHwPinNum();
    spiDevices[chip_select_pin_num].reset(new Esp32SPIDevice(bus.m.share(), chip_select.m.share()));
    return hal_pimpl::SPIDevice(spiDevices[chip_select_pin_num]);
}


// hal::Esp32Pin
hal_pimpl::Esp32Pin::Esp32Pin(uint8_t pin) {
    this->pin = static_cast<gpio_num_t>(pin);
}

uint8_t hal_pimpl::Esp32Pin::getPinNum() const {
    return this->pin;
}

gpio_num_t hal_pimpl::Esp32Pin::getHwPinNum() const {
    return this->pin;
}


// hal::Esp32GPIOPin
hal_pimpl::Esp32GPIOPin::Esp32GPIOPin(shared_ptr<Esp32Pin> &pin) : pin { pin } {
    setupGpioHardware();
}

hal_pimpl::Esp32GPIOPin::Esp32GPIOPin(shared_ptr<Esp32Pin> &&pin) : pin { pin }{
    setupGpioHardware();
}

void hal_pimpl::Esp32GPIOPin::setupGpioHardware() const {
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

void hal_pimpl::Esp32GPIOPin::high() {
    gpio_set_level(this->pin->getHwPinNum(), 1);
}

void hal_pimpl::Esp32GPIOPin::low() {
    gpio_set_level(this->pin->getHwPinNum(), 0);
}


// hal::Esp32SPIBus
hal_pimpl::Esp32SPIBus::Esp32SPIBus(shared_ptr<hal_pimpl::Esp32Pin> &&mosi, shared_ptr<hal_pimpl::Esp32Pin> &&miso,
                                    shared_ptr<hal_pimpl::Esp32Pin> &&clock) : bus_num { VSPI_HOST }, mosi { mosi }, miso { miso }, clock { clock } {
    // Do SPI Bus setup here
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

spi_host_device_t hal_pimpl::Esp32SPIBus::getBusNum() {
    return bus_num;
}


// hal::Esp32SPIDevice
hal_pimpl::Esp32SPIDevice::Esp32SPIDevice(shared_ptr<hal_pimpl::Esp32SPIBus> &&bus, shared_ptr<hal_pimpl::Esp32Pin> &&chip_select) : bus { bus }, chip_select { chip_select } {
}


// hal::HardwareContext
hal_pimpl::HardwareContext::HardwareContext() = default;

hal_pimpl::Pin hal_pimpl::HardwareContext::pin(uint8_t pin) {
    return m->pin(pin);
}

hal_pimpl::GPIOPin hal_pimpl::HardwareContext::gpioPin(Pin &pin) {
    return m->gpioPin(pin);
}

hal_pimpl::GPIOPin hal_pimpl::HardwareContext::gpioPin(uint8_t pin) {
    return m->gpioPin(pin);
}

hal_pimpl::SPIBus hal_pimpl::HardwareContext::spiBus(hal_pimpl::Pin &mosi, hal_pimpl::Pin &miso, hal_pimpl::Pin &clock) {
    return m->spiBus(mosi, miso, clock);
}

hal_pimpl::SPIBus hal_pimpl::HardwareContext::spiBus(spi_bus_num bus) {
    return m->spiBus(bus);
}

hal_pimpl::SPIDevice hal_pimpl::HardwareContext::spiDevice(SPIBus &bus, hal_pimpl::Pin &chip_select) {
    return m->spiDevice(bus, chip_select);
}

hal_pimpl::HardwareContext::~HardwareContext() = default;


// hal::Pin
hal_pimpl::Pin::Pin(shared_ptr<Esp32Pin>& pin) : m { pin } { }

uint8_t hal_pimpl::Pin::getPinNum() const {
    return m->getPinNum();
}

hal_pimpl::Pin::~Pin() = default;


// hal::GPIOPin
hal_pimpl::GPIOPin::GPIOPin(shared_ptr<Esp32GPIOPin> &pin) : m { pin } {}

hal_pimpl::GPIOPin::~GPIOPin() = default;

void hal_pimpl::GPIOPin::high() {
    m->high();
}

void hal_pimpl::GPIOPin::low() {
    m->low();
}


// hal::SPIBus
hal_pimpl::SPIBus::SPIBus(shared_ptr<Esp32SPIBus> &bus) : m { bus } {}

hal_pimpl::SPIBus::~SPIBus() = default;


// hal::SPIDevice
hal_pimpl::SPIDevice::SPIDevice(std::shared_ptr<hal_pimpl::Esp32SPIDevice> &device) : m { device } {}

unique_ptr<vector<uint8_t>> hal_pimpl::SPIDevice::transfer(const vector<uint8_t> &tx) const {
    // TODO: IMPLEMENT
    return unique_ptr<vector<uint8_t>>();
}

hal_pimpl::SPIDevice::~SPIDevice() = default;
