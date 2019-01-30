#add_library(hal STATIC GpioPin.cpp)

#target_include_directories(hal PUBLIC include)

set(COMPONENT_SRCS "GpioPin.cpp" "I2SBus.cpp")
#set(COMPONENT_SRCS "I2SBus.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "include")
register_component()
