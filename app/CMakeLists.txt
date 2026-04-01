cmake_minimum_required(VERSION 3.22)

enable_language(C CXX ASM)

add_subdirectory(Driver/FDCAN)
add_subdirectory(Driver/FLASH)
add_subdirectory(Driver/USB)

add_subdirectory(main_FW)

set(app_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(app_App_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/HEADER_INCLUDE.cpp
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${app_App_Dirs})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${app_Include_Dirs})
