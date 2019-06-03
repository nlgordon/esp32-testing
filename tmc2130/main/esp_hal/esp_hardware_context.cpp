#include "esp_hal.h"
#include "pin.h"
#include "gpiopin.h"
#include "spi.h"

using namespace hal;
using namespace esp_hal;

HardwareContext* ContextFactory::createContext() {
    return new Esp32HardwareContext();
}

Esp32HardwareContext::Esp32HardwareContext() : pins{40}, gpioPins{40}, spiBuses{3}, spiDevices{40} {

}

Pin* Esp32HardwareContext::pin(uint8_t pin) {
    if (!pins[pin]) {
        pins[pin] = std::make_unique<Esp32Pin>(*this, pin);
    }

    return pins[pin].get();
}

GPIOPin* Esp32HardwareContext::gpioPin(Pin *pin) {
    auto espPin = dynamic_cast<Esp32Pin*>(pin);
    uint8_t pin_num = espPin->getPinNum();

    if (!gpioPins[pin_num]) {
        gpioPins[pin_num] = std::make_unique<Esp32GPIOPin>(*this, espPin);
    }
    return gpioPins[pin_num].get();
}

GPIOPin* Esp32HardwareContext::gpioPin(uint8_t pin) {
    return gpioPin(this->pin(pin));
}

SPIBus* Esp32HardwareContext::spiBus(spi_bus_num bus) {
    spi_host_device_t hw_bus = bus == BUS_1 ? HSPI_HOST : VSPI_HOST;

    if (!spiBuses[hw_bus]) {
        spiBuses[hw_bus] = std::make_unique<Esp32SPIBus>(*this, hw_bus);
    }
    return spiBuses[hw_bus].get();
}

SPIDevice* Esp32HardwareContext::spiDevice(SPIBus *bus, Pin *chip_select) {
    uint8_t chip_select_pin = chip_select->getPinNum();

    if (!spiDevices[chip_select_pin]) {
        spiDevices[chip_select_pin] = std::make_unique<Esp32SPIDevice>(*this,
                dynamic_cast<Esp32SPIBus*>(bus),
                dynamic_cast<Esp32Pin*>(chip_select));
    }

    return spiDevices[chip_select_pin].get();
}

Esp32HardwareContext::~Esp32HardwareContext() = default;
