#pragma once
#ifndef TMC_2130_ESP_HAL_PIN_H
#define TMC_2130_ESP_HAL_PIN_H

#include "esp_hal.h"
#include <driver/gpio.h>

namespace esp_hal {
    class Esp32Pin : public hal::Pin {
        gpio_num_t pin;
        Esp32HardwareContext& ctx;
    public:
        explicit Esp32Pin(Esp32HardwareContext &ctx, uint8_t pin);
        uint8_t getPinNum() const override;
        gpio_num_t getHwPinNum() const;
    };
}

#endif //TMC_2130_ESP_HAL_PIN_H
