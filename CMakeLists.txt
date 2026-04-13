cmake_minimum_required(VERSION 3.13)

# PICO_SDK_PATH must be set in environment or passed via -DPICO_SDK_PATH=...
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(fly_adxl345_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(fly_adxl345_firmware
    main.c
    usb_descriptors.c
)

target_include_directories(fly_adxl345_firmware PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(fly_adxl345_firmware
    pico_stdlib
    pico_multicore
    hardware_spi
    tinyusb_device
    tinyusb_board
)

# stdio_usb disabled — direct TinyUSB used instead
pico_enable_stdio_usb(fly_adxl345_firmware 0)
pico_enable_stdio_uart(fly_adxl345_firmware 0)

# Generate .uf2 / .bin / .hex / .map
pico_add_extra_outputs(fly_adxl345_firmware)
