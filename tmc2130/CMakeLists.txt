# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set (CMAKE_CXX_STANDARD 17)
project(tmc-2130 CXX)
set(CMAKE_TOOLCHAIN_FILE ${IDF_PATH}/tools/cmake/toolchain-esp32.cmake)
include($ENV{IDF_PATH}/tools/cmake/idf.cmake)

idf_build_process(esp32
        BUILD_DIR ${CMAKE_BINARY_DIR})

add_subdirectory(main)

idf_build_executable(main)