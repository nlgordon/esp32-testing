add_library(esp_hal pin.cpp esp_hardware_context.cpp gpiopin.cpp gpiopin.h spi.cpp spi.h)

target_link_libraries(esp_hal idf::esp32 idf::freertos idf:cxx hal)