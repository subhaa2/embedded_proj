# CMake project name for the LoRa Bridge application (Pico #1 - Consolidated Sensors)
cmake_minimum_required(VERSION 3.13)

# Add this line to explicitly set the path before calling the initialization.
# This fixes the FATAL_ERROR: include could not find requested file: pico_sdk_init
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

# Create the executable named 'lora_bridge_app' from the source file 'lora_bridge.c'
# Note: You should rename 'lora_bridge.c' to 'main.c' if you are using the consolidated code.
project(Merge LANGUAGES C CXX ASM)

# Initialize the Pico SDK
# This call is now possible because pico_sdk_init.cmake has been included above.
pico_sdk_init()

add_executable(Merge
    main.c
    kissfft/kiss_fft.c
)

# Link necessary libraries and dependencies
target_link_libraries(Merge
    pico_stdlib
    hardware_uart     # For LoRa Communication
    hardware_i2c      # REQUIRED for the MLX90614 Temperature Sensor
    hardware_adc      # REQUIRED for the Analog Microphone
    pico_time         # For non-blocking timing functions
    m
)

# Enable USB serial output for PC communication (stdio_init_all in main.c)
pico_enable_stdio_usb(Merge 1)

# Add boot settings
pico_add_extra_outputs(Merge)