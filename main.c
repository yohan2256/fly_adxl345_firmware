/**
 * Fly-ADXL345-USB High Speed Firmware
 * FIX: ADXL345 FIFO read — 6-byte burst per sample (each pops one entry)
 * IMPROVED:
 *   - Dual-core: Core 0 = USB service, Core 1 = sensor read
 *   - 8 KB SPSC ring buffer (lock-free) decouples sensor from USB
 *   - Direct TinyUSB (no stdio_usb) → non-blocking writes
 *   - Batch flush (256 B threshold) for Android USB efficiency
 *   - Adaptive sleep based on FIFO fill level
 *   - flags bit1: ring-buffer drop indicator for receiver
 */

#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "tusb.h"

// ------------------------------------------------------------------ Pins ---
#define PIN_CS   9
#define PIN_SCK  10
#define PIN_MOSI 11
#define PIN_MISO 12
#define SPI_PORT spi1

// -------------------------------------------------------- ADXL345 registers --
#define REG_BW_RATE     0x2C
#define REG_POWER_CTL   0x2D
#define REG_DATA_FORMAT 0x31
#define REG_DATAX0      0x32
#define REG_FIFO_CTL    0x38
#define REG_FIFO_STATUS 0x39

// -------------------------------------------------------- Stream parameters --
#define ODR_3200HZ        0x0F
#define FIFO_WATERMARK    16      // 1..31
#define MAX_SAMPLES_FRAME 32      // <= 32
#define LOOP_SLEEP_US     200     // base sleep (us)

// ----------------------------------------------------------------- Structs --
#pragma pack(push, 1)
typedef struct {
    uint8_t  sync1;      // 0xFA
    uint8_t  sync2;      // 0xCE
    uint16_t seq_start;  // TX sequence start (LE)
    uint8_t  count;      // number of samples in frame
    uint8_t  flags;      // bit0: FIFO overrun  bit1: ring-buf drop
} frame_header_t;

typedef struct {
    int16_t x, y, z;
} sample_xyz_t;
#pragma pack(pop)

// ===========================================================================
// SPSC ring buffer (Single-Producer/Single-Consumer, lock-free)
//
// Core 1 = producer (g_head), Core 0 = consumer (g_tail).
// __sync_synchronize() barriers prevent memory-access reordering on
// Cortex-M0+ across the two RP2040 cores.
// RING_SIZE must be a power of 2.
// ===========================================================================
#define RING_SIZE 8192u          // 8 KB ~= 400 ms buffer at 20 KB/s
#define RING_MASK (RING_SIZE - 1u)

static uint8_t           g_ring[RING_SIZE];
static volatile uint32_t g_head = 0;  // producer index (Core 1)
static volatile uint32_t g_tail = 0;  // consumer index (Core 0)

static inline uint32_t ring_used(void) { return g_head - g_tail; }
static inline uint32_t ring_free(void) { return RING_SIZE - ring_used(); }

/** Core 1 only: push one frame. Returns false (drop) if buffer is full. */
static bool ring_push(const uint8_t *src, uint32_t len)
{
    if (ring_free() < len) return false;
    for (uint32_t i = 0; i < len; i++)
        g_ring[(g_head + i) & RING_MASK] = src[i];
    __sync_synchronize();   // data written before head advances
    g_head += len;
    return true;
}

/** Core 0 only: pointer to a contiguous readable region. */
static uint32_t ring_peek_contig(const uint8_t **out)
{
    uint32_t used = ring_used();
    if (!used) { *out = NULL; return 0; }
    __sync_synchronize();           // head read before data read
    uint32_t tail_idx = g_tail & RING_MASK;
    uint32_t contig   = RING_SIZE - tail_idx;
    *out = &g_ring[tail_idx];
    return (used < contig) ? used : contig;
}

/** Core 0 only: advance tail after consuming n bytes. */
static inline void ring_consume(uint32_t n)
{
    __sync_synchronize();   // data read before tail advances
    g_tail += n;
}

// ===========================================================================
// SPI helpers  (Core 1 exclusive after multicore_launch_core1)
// ===========================================================================
static inline void cs_select(void) {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);
    asm volatile("nop \n nop \n nop");
}
static inline void cs_deselect(void) {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}
static inline void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = { reg, data };
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect();
}
static inline void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= 0x80u;
    if (len > 1) reg |= 0x40u;
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, buf, len);
    cs_deselect();
}
static inline void read_one_xyz6(uint8_t out6[6]) {
    read_registers(REG_DATAX0, out6, 6); // pops one FIFO sample
}

// ===========================================================================
// CRC16-CCITT
// ===========================================================================
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

// ===========================================================================
// Core 0: USB service
//
// tud_task() MUST run on Core 0 (USB IRQ bound to Core 0).
// tud_cdc_write() puts data into TinyUSB's internal TX FIFO (non-blocking).
// tud_cdc_write_flush() marks the FIFO as ready for the next USB transfer.
//
// Batch-flush strategy: accumulate USB_FLUSH_BYTES before flushing.
// Fewer USB transactions => better throughput on Android (1 ms polling).
// ===========================================================================
#define USB_FLUSH_BYTES 256u

static void usb_service(void)
{
    tud_task();     // TinyUSB event processing (mandatory, Core 0 only)

    if (!tud_cdc_connected()) return;

    const uint8_t *ptr;
    uint32_t chunk = ring_peek_contig(&ptr);
    if (!chunk) return;

    uint32_t avail = tud_cdc_write_available();
    if (chunk > avail) chunk = avail;
    if (!chunk) return;             // TinyUSB TX FIFO full — retry next call

    uint32_t written = tud_cdc_write(ptr, chunk);
    ring_consume(written);

    // Flush when ring drained or TinyUSB TX FIFO is nearly full
    if (ring_used() == 0 || tud_cdc_write_available() < USB_FLUSH_BYTES)
        tud_cdc_write_flush();
}

// ===========================================================================
// Core 1: ADXL345 sensor loop
//
// Reads FIFO, packs frames, pushes to ring buffer.
// No USB calls here — USB latency cannot block sensor reading.
// ===========================================================================
static void core1_sensor_loop(void)
{
    static uint8_t raw[6 * MAX_SAMPLES_FRAME];
    static uint8_t frame_buf[sizeof(frame_header_t)
                              + MAX_SAMPLES_FRAME * sizeof(sample_xyz_t) + 2];

    uint16_t seq_tx     = 0;
    uint32_t drop_count = 0;

    while (true) {
        // ---- Poll FIFO status ----
        uint8_t fifo_status = 0;
        read_registers(REG_FIFO_STATUS, &fifo_status, 1);
        uint8_t entries = fifo_status & 0x3Fu;
        uint8_t overrun  = (uint8_t)(fifo_status >> 7u);

        if (!overrun && entries < FIFO_WATERMARK) {
            sleep_us(LOOP_SLEEP_US);
            continue;
        }

        // ---- Read samples (6 bytes per call = 1 FIFO sample popped) ----
        uint8_t n = (entries > MAX_SAMPLES_FRAME) ? MAX_SAMPLES_FRAME : entries;
        for (uint8_t i = 0; i < n; i++)
            read_one_xyz6(&raw[6 * i]);

        // ---- Build frame ----
        frame_header_t hdr = {
            .sync1     = 0xFA,
            .sync2     = 0xCE,
            .seq_start = seq_tx,
            .count     = n,
            .flags     = (uint8_t)(overrun | (drop_count ? 0x02u : 0x00u)),
        };

        size_t off = 0;
        memcpy(frame_buf + off, &hdr, sizeof(hdr));
        off += sizeof(hdr);

        sample_xyz_t *s = (sample_xyz_t *)(frame_buf + off);
        for (uint8_t i = 0; i < n; i++) {
            const uint8_t *p = &raw[6 * i];
            s[i].x = (int16_t)((uint16_t)p[1] << 8 | p[0]);
            s[i].y = (int16_t)((uint16_t)p[3] << 8 | p[2]);
            s[i].z = (int16_t)((uint16_t)p[5] << 8 | p[4]);
        }
        off += (size_t)n * sizeof(sample_xyz_t);

        uint16_t crc = crc16_ccitt(frame_buf, off);
        frame_buf[off++] = (uint8_t)(crc & 0xFFu);
        frame_buf[off++] = (uint8_t)(crc >> 8u);

        // ---- Push to ring buffer (non-blocking, never stalls) ----
        if (!ring_push(frame_buf, (uint32_t)off))
            drop_count++;   // ring full: USB host too slow
        else
            drop_count = 0;

        seq_tx += n;

        // ---- Adaptive sleep ----
        // Overrun or FIFO nearly full: no sleep, drain immediately
        // Otherwise: halved sleep to reduce busy-loop overhead
        if (!overrun && entries <= FIFO_WATERMARK + 4u)
            sleep_us(LOOP_SLEEP_US / 2u);   // 200 -> 100 us
    }
}

// ===========================================================================
// main (Core 0)
// ===========================================================================
int main(void)
{
    // TinyUSB init (replaces stdio_init_all + stdio_usb)
    // Requires usb_descriptors.c + tusb_config.h
    tusb_init();

    // SPI init — must complete BEFORE multicore_launch_core1
    spi_init(SPI_PORT, 5000u * 1000u);  // 5 MHz
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    sleep_ms(100);  // power-on settle

    // ADXL345 config
    write_register(REG_DATA_FORMAT, 0x0Bu);                             // FULL_RES +/-16g
    write_register(REG_BW_RATE,     ODR_3200HZ);                        // 3200 Hz
    write_register(REG_FIFO_CTL,    0x80u | (FIFO_WATERMARK & 0x1Fu)); // STREAM + watermark
    write_register(REG_POWER_CTL,   0x08u);                             // Measurement mode

    // Launch Core 1 (sensor read; SPI exclusive to Core 1 from here)
    multicore_launch_core1(core1_sensor_loop);

    // Core 0: USB tight loop — no sleep (tud_task needs frequent calls)
    while (true) {
        usb_service();
    }
}
