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

# Stock SDK USB CDC stdio — the standard, known-working implementation.
# This is the whole point of this test: no custom TinyUSB descriptors,
# no custom main-loop, just "does USB CDC enumerate on this board at all."
pico_enable_stdio_usb(usb_sanity_test 1)
pico_enable_stdio_uart(usb_sanity_test 0)

pico_add_extra_outputs(usb_sanity_test)
