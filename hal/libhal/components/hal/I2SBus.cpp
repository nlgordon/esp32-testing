#include <hal/I2SBus.h>
#include <freertos/FreeRTOS.h>
#include <driver/i2s.h>
#include "esp_system.h"

I2SBus::I2SBus() {
    i2s_config_t config = {
            .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
            .sample_rate = 36000,
            .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
            .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
            .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
            .dma_buf_count = 6,
            .dma_buf_len = 60,
            .use_apll = false,
            .fixed_mclk = 0
    };
    i2s_driver_install(I2S_NUM_0, &config, 0, nullptr);
    i2s_pin_config_t pinConfig = {
            .bck_io_num = 26,
            .ws_io_num = 25,
            .data_out_num = 22,
            .data_in_num = -1
    };
    i2s_set_pin(I2S_NUM_0, &pinConfig);
}
