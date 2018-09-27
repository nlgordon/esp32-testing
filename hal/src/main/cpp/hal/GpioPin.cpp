
#include "GpioPin.h"
#include <driver/gpio.h>

GpioPin::GpioPin(int pin) {
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = (1ULL<<pin);
    gpio_config(&io_conf);
}
