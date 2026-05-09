cmake_minimum_required(VERSION 3.22)

enable_language(C CXX ASM)

set(mXCAN_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(mXCAN_App_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/app.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/driver_fdcan.cpp
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${mXCAN_App_Dirs})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${mXCAN_Include_Dirs})

target_link_libraries(app PRIVATE gn10_can)
target_link_libraries(app PRIVATE stm32cubemx)