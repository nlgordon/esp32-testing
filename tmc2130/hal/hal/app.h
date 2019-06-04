
#ifndef TMC_2130_APP_H
#define TMC_2130_APP_H

namespace hal {
    class App {
        virtual void setup() = 0;
        virtual void loop() = 0;
    };
}

#endif //TMC_2130_APP_H
