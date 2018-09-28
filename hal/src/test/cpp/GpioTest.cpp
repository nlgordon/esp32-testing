#include "gtest/gtest.h"
#include <driver/gpio.h>
#include "hal/GpioPin.h"

gpio_config_t internal;
int pin_level = -1;
gpio_num_t pin_set = GPIO_NUM_MAX;

esp_err_t gpio_config(const gpio_config_t *pGPIOConfig) {
    esp_err_t ret;
    memcpy(&internal, pGPIOConfig, sizeof(gpio_config_t));
    return ret;
}

esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level) {
    esp_err_t ret;
    pin_level = level;
    pin_set = gpio_num;
    return ret;
}

namespace {
    class GpioTest : public ::testing::Test { };

    TEST_F(GpioTest, ConfigPin0ResultsInBitMaskOf1) {
        GpioPin pin(0);
        EXPECT_EQ(internal.pin_bit_mask, 0x1);
    }

    TEST_F(GpioTest, ConfigPin1ResultsInBitMasOf2) {
        GpioPin pin(1);
        EXPECT_EQ(internal.pin_bit_mask, 0x2);
    }

    TEST_F(GpioTest, ConfigDisablesInterruptsByDefault) {
        GpioPin pin(1);
        EXPECT_EQ(internal.intr_type, GPIO_INTR_DISABLE);
    }

    TEST_F(GpioTest, ConfigSetsModeToOutputByDefault) {
        GpioPin pin(1);
        EXPECT_EQ(internal.mode, GPIO_MODE_OUTPUT);
    }

    TEST_F(GpioTest, ConfigDisablesPullDownByDefault) {
        GpioPin pin(1);
        EXPECT_EQ(internal.pull_down_en, GPIO_PULLDOWN_DISABLE);
    }

    TEST_F(GpioTest, ConfigDisablesPullUpByDefault) {
        GpioPin pin(1);
        EXPECT_EQ(internal.pull_up_en, GPIO_PULLUP_DISABLE);
    }

    TEST_F(GpioTest, TransitioningToHighSetsLevelToHigh) {
        GpioPin pin(1);
        pin.high();
        EXPECT_EQ(pin_level, 1);
    }

    TEST_F(GpioTest, TransitioningToHighOnPin1SetsLevelOnPin1) {
        GpioPin pin(1);
        pin.high();
        EXPECT_EQ(pin_set, GPIO_NUM_1);
    }

    TEST_F(GpioTest, TransitioningToHighOnPin2SetsLevelOnPin2) {
        GpioPin pin(2);
        pin.high();
        EXPECT_EQ(pin_set, GPIO_NUM_2);
    }

    TEST_F(GpioTest, TransitioningToLowSetsLevelToLow) {
        GpioPin pin(1);
        pin.low();
        EXPECT_EQ(pin_level, 0);
    }

    TEST_F(GpioTest, TransitioningToLowOnPin1SetsLevelOnPin1) {
        GpioPin pin(1);
        pin.low();
        EXPECT_EQ(pin_set, GPIO_NUM_1);
    }

    TEST_F(GpioTest, TransitioningToLowOnPin2SetsLevelOnPin2) {
        GpioPin pin(2);
        pin.low();
        EXPECT_EQ(pin_set, GPIO_NUM_2);
    }
}
