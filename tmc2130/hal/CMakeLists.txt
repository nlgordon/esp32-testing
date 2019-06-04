add_library(hal hal/hal.h hal/pin.h hal/gpiopin.h hal/spi.h hal/tasks.h hal.cpp hal/app.h)

target_include_directories(hal PUBLIC .)