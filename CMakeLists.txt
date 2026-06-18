cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME explorer_26)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
# ===== 用户源码（底盘 + 地图/障碍/主任务；外设与 I2C 已剔除）=====
file(GLOB_RECURSE USER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/App/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Math/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Motor/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Sensor/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Task/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Driver/*.c
)
# uart.c：旧 ALIENTEK 串口驱动，与 CubeMX 的 USART1_IRQHandler 冲突且当前未使用，暂不编译
list(REMOVE_ITEM USER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Driver/uart.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/App
    ${CMAKE_CURRENT_SOURCE_DIR}/App/barrier
    ${CMAKE_CURRENT_SOURCE_DIR}/App/chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/App/map
    ${CMAKE_CURRENT_SOURCE_DIR}/Math
    ${CMAKE_CURRENT_SOURCE_DIR}/Motor
    ${CMAKE_CURRENT_SOURCE_DIR}/Sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/Task
    ${CMAKE_CURRENT_SOURCE_DIR}/Driver
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
