cmake_minimum_required(VERSION 3.22)

enable_language(C CXX ASM)

set(mUSB_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(mUSB_App_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/mUSB_HS.cpp
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${mUSB_App_Dirs})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${mUSB_Include_Dirs})