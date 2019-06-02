#include "hal_pointer.h"
#include <driver/gpio.h>
#include <driver/spi_master.h>

using namespace std;
using namespace hal_pointer;

class hal_pointer::Esp32Pin : public hal_pointer::Pin {
    gpio_num_t pin;
public:
    explicit Esp32Pin(uint8_t pin);
    uint8_t getPinNum() const override;
    gpio_num_t getHwPinNum() const;
};

hal_pointer::HardwareContext::HardwareContext() : pins{40} {

}

hal_pointer::Pin *hal_pointer::HardwareContext::pin(uint8_t pin) {
    if (!pins[pin]) {
        pins[pin] = std::make_unique<Esp32Pin>(pin);
    }

    return pins[pin].get();
}

hal_pointer::HardwareContext::~HardwareContext() = default;
// hal::Esp32Pin
hal_pointer::Esp32Pin::Esp32Pin(uint8_t pin) {
    this->pin = static_cast<gpio_num_t>(pin);
}

uint8_t hal_pointer::Esp32Pin::getPinNum() const {
    return this->pin;
}

gpio_num_t hal_pointer::Esp32Pin::getHwPinNum() const {
    return this->pin;
}
