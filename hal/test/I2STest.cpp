#include "base.h"
#include "gtest/gtest.h"
#include <driver/i2s.h>
#include "hal/I2SBus.h"
#include "esp_system.h"

namespace {
    class I2STest : public ::testing::Test {
    public:
        i2s_config_t busConfig;
        i2s_port_t port;
        i2s_port_t pinPort;
        i2s_pin_config_t pinConfig;

        I2STest();
        ~I2STest();
    };

    I2STest* testHandle;

    I2STest::I2STest() {
        testHandle = this;
    }

    I2STest::~I2STest() {
        testHandle = nullptr;
    }

    TEST_F(I2STest, ConfigSetsModeToMaster) {
        I2SBus bus;
        EXPECT_EQ(busConfig.mode & I2S_MODE_MASTER, I2S_MODE_MASTER);
    }

    TEST_F(I2STest, ConfigSetsModeTx) {
        I2SBus bus;
        EXPECT_EQ(busConfig.mode & I2S_MODE_TX, I2S_MODE_TX);
    }

    TEST_F(I2STest, ConfigSetsBitsPerSampleTo16) {
        I2SBus bus;
        EXPECT_EQ(busConfig.bits_per_sample, I2S_BITS_PER_SAMPLE_16BIT);
    }

    TEST_F(I2STest, ConfigSetsChannelFormatToTwoChannel) {
        I2SBus bus;
        EXPECT_EQ(busConfig.channel_format, I2S_CHANNEL_FMT_RIGHT_LEFT);
    }

    TEST_F(I2STest, ConfigSetsCommunicationFormatToI2S) {
        I2SBus bus;
        EXPECT_EQ(busConfig.communication_format & I2S_COMM_FORMAT_I2S, I2S_COMM_FORMAT_I2S);
    }

    TEST_F(I2STest, ConfigSetsCommunicationFormatToMSB) {
        I2SBus bus;
        EXPECT_EQ(busConfig.communication_format & I2S_COMM_FORMAT_I2S_MSB, I2S_COMM_FORMAT_I2S_MSB);
    }

    TEST_F(I2STest, ConfigSetsDMABufferCountTo6) {
        I2SBus bus;
        EXPECT_EQ(busConfig.dma_buf_count, 6);
    }

    TEST_F(I2STest, ConfigSetsDMABufferLengthTo60) {
        I2SBus bus;
        EXPECT_EQ(busConfig.dma_buf_len, 60);
    }

    TEST_F(I2STest, ConfigSetsUseApllToFalse) {
        I2SBus bus;
        EXPECT_EQ(busConfig.use_apll, false);
    }

    TEST_F(I2STest, ConfigSetsInterruptLevel1) {
        I2SBus bus;
        EXPECT_EQ(busConfig.intr_alloc_flags, ESP_INTR_FLAG_LEVEL1);
    }

    TEST_F(I2STest, ConfigSetsSampleRateTo36k) {
        I2SBus bus;
        EXPECT_EQ(busConfig.sample_rate, 36000);
    }

    TEST_F(I2STest, ConfigSetsFixedMClockToZero) {
        I2SBus bus;
        EXPECT_EQ(busConfig.fixed_mclk, 0);
    }

    TEST_F(I2STest, DriverInstallUsesI2SPort0) {
        I2SBus bus;
        EXPECT_EQ(port, I2S_NUM_0);
    }

    TEST_F(I2STest, SettingPinUsesPort0) {
        I2SBus bus;
        EXPECT_EQ(pinPort, I2S_NUM_0);
    }

    TEST_F(I2STest, SettingPinConfigSetsBckIoPinTo26) {
        I2SBus bus;
        EXPECT_EQ(pinConfig.bck_io_num, 26);
    }

    TEST_F(I2STest, SettingPinConfigSetsWsIoPinTo25) {
        I2SBus bus;
        EXPECT_EQ(pinConfig.ws_io_num, 25);
    }

    TEST_F(I2STest, SettingPinConfigSetsDataOutPinTo22) {
        I2SBus bus;
        EXPECT_EQ(pinConfig.data_out_num, 22);
    }

    TEST_F(I2STest, SettingPinConfigSetsDataInPinToNegative1) {
        I2SBus bus;
        EXPECT_EQ(pinConfig.data_in_num, -1);
    }
}

esp_err_t i2s_driver_install(i2s_port_t i2s_num, const i2s_config_t *i2s_config, int queue_size, void* i2s_queue) {
    memcpy(&testHandle->busConfig, i2s_config, sizeof(i2s_config_t));
    testHandle->port = i2s_num;
}

esp_err_t i2s_set_pin(i2s_port_t i2s_num, const i2s_pin_config_t *pin) {
    testHandle->pinPort = i2s_num;
    memcpy(&testHandle->pinConfig, pin, sizeof(i2s_pin_config_t));
}
