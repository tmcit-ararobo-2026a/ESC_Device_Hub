cmake_minimum_required(VERSION 3.22)

enable_language(C CXX ASM)

set(mmain_FW_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(mmain_FW_App_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/main_FW.cpp
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${mmain_FW_App_Dirs})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${mmain_FW_Include_Dirs})