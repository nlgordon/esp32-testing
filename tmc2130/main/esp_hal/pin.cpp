#include "pin.h"

using namespace esp_hal;
using namespace hal;

Esp32Pin::Esp32Pin(Esp32HardwareContext &ctx, uint8_t pin) : ctx{ctx} {
    this->pin = static_cast<gpio_num_t>(pin);
}

uint8_t Esp32Pin::getPinNum() const {
    return this->pin;
}

gpio_num_t Esp32Pin::getHwPinNum() const {
    return this->pin;
}

