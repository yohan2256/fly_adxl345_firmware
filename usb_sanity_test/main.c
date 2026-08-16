/**
 * Minimal USB CDC sanity test.
 *
 * Uses the Pico SDK's own stdio_usb (pico_enable_stdio_usb), which is the
 * standard, widely-used CDC implementation the SDK ships — not the custom
 * TinyUSB setup in the main fly_adxl345_firmware project. If this doesn't
 * enumerate as a COM port on the board, the problem is not in the main
 * project's application code — it's something more fundamental (board
 * wiring, RP2040 USB hardware, or the build/flash environment).
 */
#include "pico/stdlib.h"
#include <stdio.h>

int main(void)
{
    stdio_init_all();

    uint32_t count = 0;
    while (true) {
        printf("usb_sanity_test alive, count=%lu\n", (unsigned long)count++);
        sleep_ms(500);
    }
}
