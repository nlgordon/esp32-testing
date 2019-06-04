
#ifndef TMC_2130_ESP_HAL_GPIOPIN_H
#define TMC_2130_ESP_HAL_GPIOPIN_H

#include "esp_hal.h"
#include "pin.h"

namespace esp_hal {
    class Esp32GPIOPin : public hal::GPIOPin {
        Esp32Pin* pin;
        Esp32HardwareContext* ctx;
        void setupGpioHardware() const;

    public:
        explicit Esp32GPIOPin(Esp32HardwareContext* ctx, Esp32Pin *pin);
        void high() override;
        void low() override;
    };

}

#endif //TMC_2130_ESP_HAL_GPIOPIN_H
