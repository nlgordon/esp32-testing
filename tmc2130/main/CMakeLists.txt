add_executable(main main.cpp)

add_subdirectory(hal)
add_subdirectory(esp_hal)


target_link_libraries(main esp_hal)
