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
set(CMAKE_PROJECT_NAME burger)

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
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
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

# Application and micro-ROS
set(MICROROS_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_stm32cubemx_utils"
)

# Project-owned library output survives replacing the upstream utilities folder.
set(MICROROS_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake/microros/libmicroros")
if(NOT EXISTS "${MICROROS_LIBRARY_DIR}/libmicroros.a")
    message(FATAL_ERROR
        "micro-ROS library is missing. Run: bash scripts/build_microros.sh")
endif()

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    MyApp/Src/mpu9250.c
    MyApp/Src/gyro_calibration.c
    MyApp/Src/imu_attitude.c
    MyApp/Src/imu.c
    MyApp/Src/imu_diagnostics.c
    MyApp/Src/battery_monitor.c
    MyApp/Src/xl430.c
    MyApp/Src/rtos.c
    MyApp/Src/app_tasks.c
    MyApp/Src/drive_control.c
    MyApp/Src/microros_platform.c

    ${MICROROS_DIR}/extra_sources/custom_memory_manager.c
    ${MICROROS_DIR}/extra_sources/microros_allocators.c
    ${MICROROS_DIR}/extra_sources/microros_time.c
    MyApp/Src/microros_transport.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/MyApp/Inc"
    "${MICROROS_LIBRARY_DIR}/microros_include"
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    "${MICROROS_LIBRARY_DIR}/libmicroros.a"
    m
)
