#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

// RP2040
#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU    OPT_MCU_RP2040
#endif

#define CFG_TUSB_OS     OPT_OS_NONE
#define CFG_TUSB_DEBUG  0

// USB device mode only
#define CFG_TUD_ENABLED       1
#define CFG_TUD_MAX_SPEED     OPT_MODE_DEFAULT_SPEED  // Full Speed (12 Mbit/s)

// Endpoint 0 packet size — MUST be 64 for Full Speed USB.
// Used in bMaxPacketSize0 of the device descriptor.
// Missing this causes bMaxPacketSize0 = 0 and enumeration failure.
#define CFG_TUD_ENDPOINT0_SIZE  64

// CDC ACM: 1 interface
#define CFG_TUD_CDC     1
#define CFG_TUD_MSC     0
#define CFG_TUD_HID     0
#define CFG_TUD_MIDI    0
#define CFG_TUD_VENDOR  0

// CDC buffer sizes
// 512 B TX holds ~2.5 max frames (200 B each) — good for batch flushing
// 512 B RX for any host->device control (line coding, etc.)
#define CFG_TUD_CDC_RX_BUFSIZE  512
#define CFG_TUD_CDC_TX_BUFSIZE  512

#endif // _TUSB_CONFIG_H_
