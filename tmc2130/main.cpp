#include <stdio.h>
#include <string.h>
#include <iostream>
#include "hal/hal.h"
#include "hal/tasks.h"
#include "hal/app.h"

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

    class StepperApp : public hal::App{
        GPIOPin* enablePin;
        GPIOPin* directionPin;
        GPIOPin* stepPin;
        SPIDevice* spiDevice;

    public:
        StepperApp(GPIOPin* enablePin, GPIOPin* directionPin, GPIOPin* stepPin, SPIDevice* spiDevice) : enablePin {enablePin}, directionPin{directionPin}, stepPin{stepPin}, spiDevice{spiDevice} {

        };

        void setup() override {
            print_register(spiDevice, REG_GSTAT);
            print_register(spiDevice, REG_GCONF);
            print_register(spiDevice, REG_IHOLD_IRUN);
            print_register(spiDevice, REG_CHOPCONF);

            cout << "Setting current reference" << endl;
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
        }

        void loop() override {
            enablePin->high();
            directionPin->high();

            cout << "Step" << endl;
            enablePin->low();

            for (int i = 0; i < MICRO_STEPPING * STEPPER_STEPS_PER_REVOLUTION; i++) {
                stepPin->high();
                usleep(1);
                taskDelay(1);
                stepPin->low();
                usleep(1);
                taskDelay(1);
            }
            cout << "Done stepping" << endl;
            enablePin->high();
            print_register(spiDevice, REG_DRVSTATUS);
        }
    };

    void app_main(void)
    {
        cout << "Starting main" << endl;

        cout << "Starting initialization" << endl;
        auto ctx = (new ContextFactory())->createContext();
        auto spiBus = ctx->spiBus(SpiBusNum::BUS_1);

        cout << "Initialized bus and adding device" << endl;
        auto spiDevice = spiBus->device(PIN_NUM_CS);

        cout << "Configured SPI!" << endl;

        auto enablePin = ctx->gpioPin(PIN_EN);
        auto directionPin = ctx->gpioPin(PIN_DIR);
        auto stepPin = ctx->gpioPin(PIN_STEP);

        StepperApp app(enablePin, directionPin, stepPin, spiDevice);

        app.setup();

        while (true) {
            app.loop();
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
