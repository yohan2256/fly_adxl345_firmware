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

# stdio_usb는 출력 직전에 stdio_usb_connected()를 확인하고, 기본값에서 그 판정은
# CDC의 DTR 신호를 읽는다. DTR을 올리지 않는 호스트(안드로이드의
# usb-serial-for-android는 기본값이 false다)에서는 열거도 되고 포트도 열리지만
# 전송분이 통째로 버려져 수신 바이트만 0이 된다 — 원인이 드러나지 않는 실패다.
# tud_ready() 기준으로 바꿔 호스트의 DTR 처리와 무관하게 스트리밍한다.
target_compile_definitions(fly_adxl345_firmware PRIVATE
    PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1
)

# Generate .uf2 / .bin / .hex / .map
pico_add_extra_outputs(fly_adxl345_firmware)
