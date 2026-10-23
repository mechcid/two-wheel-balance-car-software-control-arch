cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME balance_car)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Debug 保留 -g3，项目及 HAL 使用 -Og，给解算和数学库留出 64 KiB Flash 空间。
target_compile_options(STM32_Drivers PRIVATE "$<$<CONFIG:Debug>:-Og>")
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE "$<$<CONFIG:Debug>:-Og>")

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    module/base/src/timebase.c
    module/base/src/TIM_IT.c
    module/overlay_app/src/motion_estimator.c
    module/overlay_app/src/motion_app.c
    module/overlay_app/src/odometry_app.c
    module/overlay_app/src/debug_app.c
    module/overlay_app/src/trace_sources.c
    module/overlay_app/src/trace_app.c
    module/overlay_app/src/lqr_app.c
    module/overlay_app/src/status_state_app.c
    module/base/src/mpu6050.c
    Core/Src/motor_board.c
    module/base/src/encoders.c
    module/overlay_app/src/motor_app.cpp
    module/base/src/scheduler.c
    module/overlay_app/src/display_app.c
    module/overlay_app/src/telemetry_app.c
    module/overlay_app/src/sensor_app.c
    module/base/src/uart_debug.c
    module/base/src/oled.c
)

# Cortex-M3 没有 FPU，解算热路径在 Debug 下也启用优化，保留调试符号。
set_source_files_properties(module/overlay_app/src/motion_estimator.c PROPERTIES COMPILE_OPTIONS "-O2")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    module/base/inc
    module/overlay_app/inc
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

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/balance_car.bin)
