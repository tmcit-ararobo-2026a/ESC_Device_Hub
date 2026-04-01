cmake_minimum_required(VERSION 3.22)

enable_language(C CXX ASM)

set(mFDCAN_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(mFDCAN_App_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/mFDCAN.cpp
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${mFDCAN_App_Dirs})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${mFDCAN_Include_Dirs})