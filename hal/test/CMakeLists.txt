cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in ${CMAKE_BINARY_DIR}/googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
    message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
    message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
#set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
        ${CMAKE_BINARY_DIR}/googletest-build
        EXCLUDE_FROM_ALL)

# Now simply link against gtest or gtest_main as needed. Eg
add_executable(test GpioTest.cpp I2STest.cpp)
#add_executable(test I2STest.cpp)

include($ENV{IDF_PATH}/tools/cmake/utilities.cmake)
add_library(esp32-system INTERFACE)
target_include_directories(esp32-system INTERFACE include-fake)
#target_include_directories(esp32-system INTERFACE $ENV{IDF_PATH}/components/freertos/include)
#target_include_directories(esp32-system INTERFACE $ENV{IDF_PATH}/components/driver/include)
#target_include_directories(esp32-system INTERFACE $ENV{IDF_PATH}/components/esp32/include)
#target_include_directories(esp32-system INTERFACE $ENV{IDF_PATH}/components/soc/esp32/include)
#target_include_directories(esp32-system INTERFACE $ENV{IDF_PATH}/components/soc/include)
#target_include_directories(esp32-system INTERFACE $ENV{IDF_PATH}/components/newlib/include)
#target_include_directories(esp32-system INTERFACE $ENV{IDF_PATH}/components/heap/include)
#target_include_directories(esp32-system INTERFACE $ENV{IDF_PATH}/components/pthread/include)

add_library(hal STATIC)
target_link_libraries(hal esp32-system)
target_link_libraries(test esp32-system)
target_link_libraries(test gtest_main)

macro(register_component)
    spaces2list(COMPONENT_SRCS)
    add_prefix(COMPONENT_SRCS "../libhal/components/hal/" ${COMPONENT_SRCS})
    target_sources(hal PRIVATE ${COMPONENT_SRCS})
    target_include_directories(hal PUBLIC "../libhal/components/hal/${COMPONENT_ADD_INCLUDEDIRS}")
    target_link_libraries(test hal)
endmacro()

include(../libhal/components/hal/CMakeLists.txt)
add_test(NAME test COMMAND test)
