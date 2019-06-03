
#ifndef TMC_2130_PIN_H
#define TMC_2130_PIN_H

#include <stdint.h>

namespace hal {
    class Pin {
    public:
        virtual uint8_t getPinNum() const = 0;
    };
}

#endif //TMC_2130_PIN_H
