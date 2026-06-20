cmake_minimum_required(VERSION 3.22)

add_library(esc_control_hub STATIC
    src/app.cpp
    src/fdcan_driver.cpp
    src/can_driver.cpp
    src/c6x_can.cpp
)

target_include_directories(esc_control_hub PUBLIC
    include
    ${CMAKE_SOURCE_DIR}/gn10_can
)
target_compile_features(esc_control_hub PUBLIC cxx_std_17)

target_link_libraries(esc_control_hub PUBLIC gn10_can)
target_link_libraries(esc_control_hub PRIVATE stm32cubemx)