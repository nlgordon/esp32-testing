# Now simply link against gtest or gtest_main as needed. Eg
add_executable(test GpioTest.cpp)
target_link_libraries(test gtest_main)
target_link_libraries(test hal)
# Steal some of the logic from the main esp-idf build to scan the components directory and pull the include dirs
target_include_directories(test PUBLIC ../../main/cpp)
target_include_directories(test PUBLIC $ENV{IDF_PATH}/components/driver/include)
target_include_directories(test PUBLIC $ENV{IDF_PATH}/components/esp32/include)
target_include_directories(test PUBLIC $ENV{IDF_PATH}/components/soc/esp32/include)
target_include_directories(test PUBLIC $ENV{IDF_PATH}/components/soc/include)
add_test(NAME test COMMAND test)
