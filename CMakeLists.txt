# CanSat Avionics - Ignitia
# CMake build for RP2040-Zero with Pico SDK
# 
# Build instructions:
#   mkdir build && cd build
#   export PICO_SDK_PATH=/path/to/pico-sdk
#   cmake ..
#   make -j4
#
# Output: build/ignitia_avionics.uf2

cmake_minimum_required(VERSION 3.13)

# Skip building picotool (not needed for UF2 generation)
set(PICO_SDK_FETCH_FROM_GIT OFF)
set(PICOTOOL_FETCH_FROM_GIT_PATH OFF)

# Initialize Pico SDK from external path or submodule
include(pico_sdk_import.cmake)

project(ignitia_avionics C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add executable with all source files
add_executable(ignitia_avionics
    src/main.c
    src/drivers/mpu6050.c
    src/drivers/bme280.c
    src/drivers/neo_m8m.c
    src/drivers/servo.c
    src/drivers/nrf_radio.c
    src/modules/flight_state.c
    src/modules/deployment.c
    src/modules/telemetry.c
    src/modules/sensor_fusion.c
    src/utils/ring_buffer.c
    src/utils/filters.c
)

# Include directories
target_include_directories(ignitia_avionics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
)

# Link required Pico SDK libraries
target_link_libraries(ignitia_avionics
    pico_stdlib
    hardware_i2c
    hardware_uart
    hardware_pwm
    hardware_timer
    hardware_gpio
    hardware_adc
    hardware_clocks
)

# Enable USB output, disable UART for stdio (we use UART1 for GPS)
pico_enable_stdio_usb(ignitia_avionics 1)
pico_enable_stdio_uart(ignitia_avionics 0)

# Create UF2 file for flashing
pico_add_extra_outputs(ignitia_avionics)

# Compiler warnings
target_compile_options(ignitia_avionics PRIVATE
    -Wall
    -Wextra
    -Werror=return-type
    -Wno-unused-parameter
)

# Optional: Build tests
option(BUILD_TESTS "Build unit tests" OFF)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()

# Print build info
message(STATUS "===========================================")
message(STATUS "  Ignitia CanSat Avionics")
message(STATUS "  Target: RP2040-Zero")
message(STATUS "  Output: ignitia_avionics.uf2")
message(STATUS "===========================================")
