cmake_minimum_required(VERSION 3.22)

enable_language(C CXX ASM)

set(mFLASH_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(mFLASH_App_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/mFLASH.cpp
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${mFLASH_App_Dirs})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${mFLASH_Include_Dirs})