cmake_minimum_required(VERSION 3.22)

enable_language(C CXX ASM)

set(APP_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(APP_App_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/app.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/driver_fdcan.cpp
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${APP_App_Dirs})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${APP_Include_Dirs})

target_link_libraries(${CMAKE_PROJECT_NAME} gn10_can)
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx)