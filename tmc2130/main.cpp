#include <stdio.h>
#include <string.h>
#include "hal/hal.h"
#include "hal/tasks.h"

#define PIN_NUM_CS   5
#define PIN_EN       21
#define PIN_DIR      32
#define PIN_STEP     33

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
using namespace hal;

void printVector(const std::vector<uint8_t> &data, const std::string &label);

unique_ptr<vector<uint8_t>> read_register(SPIDevice *spiDevice, uint8_t reg);

void print_register(SPIDevice *spiDevice, uint8_t device_register);

void print_register(SPIDevice *spiDevice, uint8_t device_register) {
    printf("Reading register %#2x : ", device_register);
    auto register_data = read_register(spiDevice, device_register);
    printVector(*register_data, "Returned data");
}

unique_ptr<vector<uint8_t>> read_register(SPIDevice *spiDevice, uint8_t reg) {
    vector<uint8_t> tx_data{(uint8_t)(READ_FLAG | reg), 0, 0, 0, 0};
    spiDevice->transfer(tx_data);
    return spiDevice->transfer(tx_data);
}


extern "C" {

void app_main(void)
{
    printf("Starting main\n");

    printf("Starting initialization\n");
    unique_ptr<HardwareContext> ctx = (new ContextFactory())->createContext();
    SPIBus* spiBus = ctx->spiBus(SpiBusNum::BUS_1);

    printf("Initialized and adding device\n");
    SPIDevice* spiDevice = ctx->spiDevice(spiBus, ctx->pin(PIN_NUM_CS));

    printf("Configured SPI!\n");

    print_register(spiDevice, REG_GSTAT);
    print_register(spiDevice, REG_GCONF);
    print_register(spiDevice, REG_IHOLD_IRUN);
    print_register(spiDevice, REG_CHOPCONF);

    printf("Setting current reference\n");
    const vector<uint8_t> gconf_reg_set = vector<uint8_t> {WRITE_FLAG | REG_GCONF, 0x00, 0x00, 0x00, 0x01};
    const vector<uint8_t> ihold_reg_set = vector<uint8_t> {WRITE_FLAG | REG_IHOLD_IRUN, 0x00, 0x00, 0x10, 0x10};
    const vector<uint8_t> chopconf_reg_set = vector<uint8_t> {WRITE_FLAG | REG_CHOPCONF, 0x00, 0x00, 0x80, 0x08};
    printVector(gconf_reg_set, "GCONF");
    spiDevice->transfer(gconf_reg_set);
    printVector(ihold_reg_set, "IHOLD");
    spiDevice->transfer(ihold_reg_set);
    printVector(chopconf_reg_set, "CHOPCONF");
    spiDevice->transfer(chopconf_reg_set);
    print_register(spiDevice, REG_GCONF);
    print_register(spiDevice, REG_IHOLD_IRUN);
    print_register(spiDevice, REG_CHOPCONF);
    print_register(spiDevice, REG_DRVSTATUS);

    GPIOPin* enablePin = ctx->gpioPin(PIN_EN);
    GPIOPin* directionPin = ctx->gpioPin(PIN_DIR);
    GPIOPin* stepPin = ctx->gpioPin(PIN_STEP);

    while (true) {
        enablePin->high();
        directionPin->high();

        printf("Step\n");
        enablePin->low();

        for (int i = 0; i < MICRO_STEPPING * STEPPER_STEPS_PER_REVOLUTION; i++) {
            stepPin->high();
            usleep(1);
            taskDelay(1);
            stepPin->low();
            usleep(1);
            taskDelay(1);
        }
        printf("Done stepping\n");
        enablePin->high();
        print_register(spiDevice, REG_DRVSTATUS);
    }
}

}

void printVector(const vector <uint8_t> &data, const std::string &label) {
    printf("%s : ", label.c_str());
    for (auto item : data) {
        printf("0x%02x ", item);
    }
    printf("\n");
}
