add_executable(main main.cpp hal.cpp hal_pimpl.cpp hal_pointer.cpp)

target_link_libraries(main idf::esp32 idf::freertos idf:cxx)
