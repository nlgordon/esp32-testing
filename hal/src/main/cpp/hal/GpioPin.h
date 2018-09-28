#ifndef HAL_GPIOPIN_H
#define HAL_GPIOPIN_H

#include "driver/gpio.h"

class GpioPin {
private:
    gpio_num_t pin;
public:
    GpioPin(int pin);
    void high();
    void low();
};


#endif //HAL_GPIOPIN_H
