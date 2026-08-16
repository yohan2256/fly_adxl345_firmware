cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(usb_sanity_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(usb_sanity_test
    main.c
)

target_link_libraries(usb_sanity_test
    pico_stdlib
)

# Diagnostic: this board's flash chip (W25Q16JVSSIQ) may not be fully
# command/timing-compatible with the default "pico" board's boot_stage2
# (boot2_w25q080), which runs from external flash on every normal boot
# BEFORE main() and before any USB code executes. BOOTSEL mode bypasses
# this entirely (pure RP2040 ROM), which is why it works while normal
# boot doesn't, regardless of what firmware is flashed. generic_03h uses
# the universally-supported legacy 0x03 read command (slower XIP, but
# compatible with virtually any SPI NOR flash) as a compatibility test.
pico_set_boot_stage2(usb_sanity_test generic_03h)

# Stock SDK USB CDC stdio — the standard, known-working implementation.
# This is the whole point of this test: no custom TinyUSB descriptors,
# no custom main-loop, just "does USB CDC enumerate on this board at all."
pico_enable_stdio_usb(usb_sanity_test 1)
pico_enable_stdio_uart(usb_sanity_test 0)

pico_add_extra_outputs(usb_sanity_test)
