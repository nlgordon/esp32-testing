#ifndef TMC_2130_HAL_POINTER_H
#define TMC_2130_HAL_POINTER_H

#include <stdint.h>
#include <vector>
#include <memory>

namespace hal_pointer {
    class Pin {
        virtual uint8_t getPinNum() const = 0;
    };

    class HardwareContext {
        std::vector<std::unique_ptr<Pin>> pins;
    public:
        HardwareContext();
        ~HardwareContext();
        Pin* pin(uint8_t pin);
    };

    class Esp32Pin;
}

#endif //TMC_2130_HAL_POINTER_H
