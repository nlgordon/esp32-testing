
#ifndef TMC_2130_GPIOPIN_H
#define TMC_2130_GPIOPIN_H

namespace hal {
    class GPIOPin {
        public:
        virtual void high() = 0;
        virtual void low() = 0;
    };
};

#endif //TMC_2130_GPIOPIN_H
