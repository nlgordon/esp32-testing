
#include "GpioPin.h"
#include <driver/gpio.h>

GpioPin::GpioPin(int pin) {
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = (1ULL<<pin);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    this->pin = static_cast<gpio_num_t>(pin);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void GpioPin::high() {
    ESP_ERROR_CHECK(gpio_set_level(this->pin, 1));
}

void GpioPin::low() {
    ESP_ERROR_CHECK(gpio_set_level(this->pin, 0));
}
