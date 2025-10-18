# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.12)

# Set the project name
set(PICO_BOARD pico)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Include the Pico SDK
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

#Define the project executable name
project(MMWAVE C CXX ASM)

pico_sdk_init()

#Add the executable using the C source file
add_executable(MMWAVE
    main.c
)

# Enable standard I/O over USB (for printing to the serial monitor)
pico_enable_stdio_usb(MMWAVE 1)
pico_enable_stdio_uart(MMWAVE 0)

# Link the necessary libraries
target_link_libraries(MMWAVE
    # Standard Pico libraries
    pico_stdlib
    # Library needed for UART communication with the radar
    hardware_uart
)

# Create the .uf2 file and other build artifacts
pico_add_extra_outputs(MMWAVE)
