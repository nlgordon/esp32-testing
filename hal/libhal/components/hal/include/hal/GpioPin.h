#ifndef HAL_GPIOPIN_H
#define HAL_GPIOPIN_H

class GpioPin {
private:
    int pin;
public:
    GpioPin(int pin);
    void high();
    void low();
};


#endif //HAL_GPIOPIN_H
