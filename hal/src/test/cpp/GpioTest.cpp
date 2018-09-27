#include "gtest/gtest.h"
#include <driver/gpio.h>
#include "hal/GpioPin.h"

gpio_config_t internal;

esp_err_t gpio_config(const gpio_config_t *pGPIOConfig) {
    memcpy(&internal, pGPIOConfig, sizeof(gpio_config_t));
}

namespace {
    class GpioTest : public ::testing::Test { };

    TEST_F(GpioTest, ConfigPinBitMask) {
        GpioPin pin(1);
        EXPECT_EQ(internal.pin_bit_mask, 0x2);
    }
}
