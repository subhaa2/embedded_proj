cmake_minimum_required(VERSION 3.13)

project(spider)
pico_sdk_init()

add_executable(spider spider_main.c)
target_link_libraries(spider pico_stdlib hardware_i2c)

pico_enable_stdio_usb(spider 1)
pico_enable_stdio_uart(spider 1)


pico_add_extra_outputs(spider)

example_auto_set_url(spider)
