
#include "gpiopin.h"

using namespace esp_hal;

Esp32GPIOPin::Esp32GPIOPin(Esp32HardwareContext* ctx, Esp32Pin *pin) : pin {pin}, ctx{ctx} {
    setupGpioHardware();
}

void Esp32GPIOPin::setupGpioHardware() const {
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

void Esp32GPIOPin::high() {
    gpio_set_level(this->pin->getHwPinNum(), 1);
}

void Esp32GPIOPin::low() {
    gpio_set_level(this->pin->getHwPinNum(), 0);
}
