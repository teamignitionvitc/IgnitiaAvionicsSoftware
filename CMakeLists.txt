cmake_minimum_required(VERSION 3.13)

# ================================
# Target Board: RP2040
# ================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Prefer the in-repo Pico SDK if present. This avoids relying on a global
# PICO_SDK_PATH environment variable (which can be stale/wrong on Windows).
if (NOT PICO_SDK_PATH AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/pico-sdk/pico_sdk_init.cmake")
    set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/pico-sdk")
endif()

# Optional: skip unnecessary downloads
set(PICO_SDK_FETCH_FROM_GIT OFF)
set(PICOTOOL_FETCH_FROM_GIT_PATH OFF)

# Import Pico SDK
include(pico_sdk_import.cmake)

project(ignitia_avionics C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Language standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize SDK
pico_sdk_init()

# ================================
# Executable
# ================================
add_executable(ignitia_avionics
    src/main.c

    # Drivers
    src/drivers/mpu6050.c
    src/drivers/bme280.c
    src/drivers/neo_m8m.c
    src/drivers/servo.c
    src/drivers/nrf_radio.c

    # Modules
    src/modules/state_machine.c
    src/modules/flight_logic.c
    src/modules/flight_state.c
    src/modules/deployment.c
    src/modules/telemetry.c
    src/modules/sensor_fusion.c
    src/modules/logger.c
    src/modules/health_monitor.c
    src/modules/event_system.c

    # Utils
    src/utils/ring_buffer.c
    src/utils/filters.c
)

# ================================
# Include Paths
# ================================
target_include_directories(ignitia_avionics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
)

# ================================
# Link Libraries
# ================================
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

# ================================
# StdIO Configuration
# ================================
pico_enable_stdio_usb(ignitia_avionics 1)
pico_enable_stdio_uart(ignitia_avionics 0)

# ================================
# Output (UF2, bin, elf)
# ================================
pico_add_extra_outputs(ignitia_avionics)

# ================================
# Compiler Options
# ================================
target_compile_options(ignitia_avionics PRIVATE
    -Wall
    -Wextra
    -Werror=return-type
    -Wno-unused-parameter
)

# ================================
# Multicore (RP2040 supports it)
# ================================
# Uncomment if using core1
# target_link_libraries(ignitia_avionics pico_multicore)

# ================================
# Tests
# ================================
option(BUILD_TESTS "Build unit tests" OFF)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()

# ================================
# Build Info
# ================================
message(STATUS "===========================================")
message(STATUS "  Ignitia CanSat Avionics")
message(STATUS "  Target: RP2040")
message(STATUS "  Output: ignitia_avionics.uf2")
message(STATUS "===========================================")