cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(legacy_reference C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(legacy_reference
    main.c
)

target_link_libraries(legacy_reference
    pico_stdlib
    hardware_spi
)

# The legacy firmware used the SDK's stock stdio_usb (stdio_init_all),
# not the custom TinyUSB descriptors the current firmware uses.
pico_enable_stdio_usb(legacy_reference 1)
pico_enable_stdio_uart(legacy_reference 0)

pico_add_extra_outputs(legacy_reference)
