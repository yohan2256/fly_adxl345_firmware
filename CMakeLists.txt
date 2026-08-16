cmake_minimum_required(VERSION 3.13)

# PICO_SDK_PATH must be set in environment or passed via -DPICO_SDK_PATH=...
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(fly_adxl345_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(fly_adxl345_firmware
    main.c
)

target_link_libraries(fly_adxl345_firmware
    pico_stdlib
    pico_multicore
    pico_bootrom
    hardware_spi
)

# Stock SDK USB CDC. Deliberately no tinyusb_device/tinyusb_board here, no
# usb_descriptors.c, and no tusb_config.h in this directory: pico_stdio_usb
# supplies its own TinyUSB configuration and descriptors. The custom
# descriptor set this project previously carried did not enumerate on the
# FLY ADXL345 board — the host saw no device at all — while this stock path
# is confirmed working on the same board, SDK and toolchain.
pico_enable_stdio_usb(fly_adxl345_firmware 1)
pico_enable_stdio_uart(fly_adxl345_firmware 0)

# Generate .uf2 / .bin / .hex / .map
pico_add_extra_outputs(fly_adxl345_firmware)
