//
// Created by nathan.l.gordon on 2019-03-29.
//

#ifndef TMC_2130_PIN_H
#define TMC_2130_PIN_H


#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <vector>
#include <memory>
#include "../../../../espressif/esp-idf/components/driver/include/driver/gpio.h"

class Pin {

    gpio_num_t pin;
};


#endif //TMC_2130_PIN_H
