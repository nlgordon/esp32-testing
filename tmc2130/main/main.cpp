#include <stdio.h>
#include <string.h>
#include <endian.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "hal.h"


#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_EN       GPIO_NUM_21
#define PIN_DIR      GPIO_NUM_32
#define PIN_STEP     GPIO_NUM_33

#define PARALLEL_LINES 16

// TMC2130 registers
#define REG_GCONF      0x00
#define REG_GSTAT      0x01
#define REG_IHOLD_IRUN 0x10
#define REG_CHOPCONF   0x6C
#define REG_COOLCONF   0x6D
#define REG_DCCTRL     0x6E
#define REG_DRVSTATUS  0x6F


#define WRITE_FLAG 1 << 7
#define READ_FLAG 0 << 7

#define STEPPER_STEPS_PER_REVOLUTION 400
#define MICRO_STEPPING 256

using namespace std;

unique_ptr<vector<uint8_t>> read_register(SPIDevice &spiDevice, uint8_t reg);

void print_register(SPIDevice &spiDevice, uint8_t device_register);

void init_spi() {
    printf("Starting initialization\n");
    SPIBus spiBus;
    htobe32(32);
    be32toh(32);

    printf("Initialized and adding device\n");
    SPIDevice spiDevice(PIN_NUM_CS, spiBus, 0);

    printf("Configured SPI!\n");

    print_register(spiDevice, REG_GSTAT);
    print_register(spiDevice, REG_GCONF);
    print_register(spiDevice, REG_IHOLD_IRUN);
    print_register(spiDevice, REG_CHOPCONF);

    printf("Setting current reference\n");
    spiDevice.transfer(vector<uint8_t> {WRITE_FLAG | REG_GCONF, 0x00, 0x00, 0x00, 0x01});
    spiDevice.transfer(vector<uint8_t> {WRITE_FLAG | REG_IHOLD_IRUN, 0x00, 0x00, 0x10, 0x10});
    spiDevice.transfer(vector<uint8_t> {WRITE_FLAG | REG_CHOPCONF, 0x00, 0x00, 0x80, 0x08});

    print_register(spiDevice, REG_GCONF);
    print_register(spiDevice, REG_IHOLD_IRUN);
    print_register(spiDevice, REG_CHOPCONF);
    print_register(spiDevice, REG_DRVSTATUS);
}

void print_register(SPIDevice &spiDevice, uint8_t device_register) {
    printf("Reading register %#2x : ", device_register);
    auto register_data = read_register(spiDevice, device_register);
    printVector(*register_data);
}

unique_ptr<vector<uint8_t>> read_register(SPIDevice &spiDevice, uint8_t reg) {
    vector<uint8_t> tx_data{(uint8_t)(READ_FLAG | reg), 0, 0, 0, 0};
    spiDevice.transfer(tx_data);
    return spiDevice.transfer(tx_data);
}


extern "C" {

void app_main(void)
{
    printf("Starting main\n");

    init_spi();

    GPIOPin enablePin(PIN_EN);
    GPIOPin directionPin(PIN_DIR);
    GPIOPin stepPin(PIN_STEP);

    enablePin.high();
    directionPin.high();

    printf("Step\n");
    enablePin.low();

    for (int i = 0; i < MICRO_STEPPING * STEPPER_STEPS_PER_REVOLUTION; i++) {
        stepPin.high();
        usleep(1);
//        delayMilliseconds(1);
        stepPin.low();
        usleep(1);
//        delayMilliseconds(1);
    }
    printf("Done stepping\n");
    enablePin.high();
}

};
