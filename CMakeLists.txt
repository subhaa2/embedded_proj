cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

set(PICO_BOARD pico)

project(embedded_proj C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(embedded_proj
    main.c
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(embedded_proj 1)
pico_enable_stdio_uart(embedded_proj 0)

# Link required libraries
target_link_libraries(embedded_proj 
    pico_stdlib
    hardware_uart
    hardware_gpio
    hardware_i2c
    hardware_adc
    hardware_timer
    pico_time
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(embedded_proj)

add_subdirectory(MMWAVE)
add_subdirectory(Merge)