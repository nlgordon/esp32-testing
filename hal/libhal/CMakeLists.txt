cmake_minimum_required(VERSION 3.5)

set(COMPONENTS driver bootloader esptool_py hal main)
#set(EXTRA_COMPONENT_DIRS components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(libhal)
