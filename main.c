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
 *   - Overrun detected via INT_SOURCE (not FIFO_STATUS bit 7, which is
 *     FIFO_TRIG and only meaningful in TRIGGER mode)
 *   - frame_header_t.dropped: exact count of samples dropped since the
 *     previous frame that reached the ring buffer
 *   - Core 1 pauses acquisition while no USB host is attached, instead of
 *     polling/dropping frames into the void
 *   - flags bit2: periodic DEVID re-read failed (proxy for SPI link
 *     integrity, since the ADXL345 has no CRC on its SPI output) — firmware
 *     auto-reconfigures the sensor when this happens
 *   - Retransmission on request: the last ~16 KB of transmitted frames are
 *     kept in a history buffer. If the host detects a bad/missing frame
 *     (CRC mismatch or a gap in seq_start), it can ask for it again by
 *     sending 4 bytes over the same CDC connection:
 *       0xF0 0x0D <seq_lo> <seq_hi>   (seq = the frame's seq_start, LE)
 *     The matching frame (if not yet evicted from history) is resent
 *     verbatim, ahead of any live data, so the host sees it as soon as
 *     possible. A request for an already-evicted or unknown seq is
 *     silently ignored — the host must decide how long to wait before
 *     giving up.
 *   - Status frames (0xFA 0xCD): the ADXL345's ODR is derived from an
 *     internal RC oscillator, factory-trimmed to only +/-2% (up to +/-5%
 *     over temperature per ADI). FS_DEFAULT = 3200.0 Hz on the host side is
 *     therefore a nominal value, not the true sample rate, and since
 *     stiffness-type results scale with fs^2 in downstream analysis, that
 *     clock error is amplified. The RP2040's crystal reference (~+/-30 ppm)
 *     is two orders of magnitude more accurate, so Core 1 times its own
 *     FIFO reads against it once a second and reports the true measured
 *     ODR to the host — a free, per-device, temperature-tracking
 *     calibration the host can use in place of the nominal constant.
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
#define REG_DEVID       0x00
#define REG_BW_RATE     0x2C
#define REG_POWER_CTL   0x2D
#define REG_INT_SOURCE  0x30
#define REG_DATA_FORMAT 0x31
#define REG_DATAX0      0x32
#define REG_FIFO_CTL    0x38
#define REG_FIFO_STATUS 0x39

// INT_SOURCE bit 0 latches on true FIFO overrun regardless of INT_ENABLE;
// FIFO_STATUS bit 7 is FIFO_TRIG (trigger-mode only), not overrun, and
// must not be used to detect overrun in STREAM mode.
#define INT_SOURCE_OVERRUN 0x01u

// The ADXL345 has no CRC/parity on its SPI output, so per-sample integrity
// cannot be verified directly. DEVID is a fixed, known value; re-reading it
// periodically is the practical proxy for "is the SPI link still sane" and
// catches wiring/timing faults that would otherwise silently corrupt samples.
#define DEVID_EXPECTED       0xE5u
#define LINK_CHECK_INTERVAL  100u   // re-verify DEVID every N transmitted frames

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
    uint8_t  flags;      // bit0: FIFO overrun  bit1: ring-buf drop  bit2: SPI link check failed
    uint16_t dropped;    // samples dropped since previous frame (saturates at 0xFFFF)
} frame_header_t;

typedef struct {
    int16_t x, y, z;
} sample_xyz_t;

// Periodic status frame: reports the ODR actually measured against the
// RP2040's crystal, in milli-Hz (Hz * 1000), so the host isn't stuck
// assuming the nominal 3200 Hz.
typedef struct {
    uint8_t  sync1;    // 0xFA
    uint8_t  sync2;    // 0xCD
    uint32_t odr_mhz;  // measured ODR, milli-Hz, LE
} status_header_t;
#pragma pack(pop)

#define FRAME_BUF_MAX_LEN (sizeof(frame_header_t) \
                            + MAX_SAMPLES_FRAME * sizeof(sample_xyz_t) + 2u)
#define STATUS_FRAME_LEN  (sizeof(status_header_t) + 2u)   // header + CRC16
#define ODR_REPORT_INTERVAL_US 1000000ull   // report measured ODR ~once/second

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

// Written by Core 0 (usb_service), read by Core 1 (core1_sensor_loop).
// A plain bool status flag; no ordering/atomicity guarantees needed beyond
// what a single-word read/write already provides on Cortex-M0+.
static volatile bool g_usb_connected = false;

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
// Retransmission history (Core 1 writes, Core 0 reads on request)
//
// Every frame that makes it into the live ring is also copied here. If the
// host later reports a bad/missing frame, it can ask for that exact
// seq_start again; we replay it from this buffer instead of the sensor,
// since the original FIFO entry is long gone by then (each SPI read pops
// it). Entries age out once HIST_SIZE bytes' worth of newer frames have
// been written, or once HIST_MAX_ENTRIES newer frames exist.
//
// This is deliberately best-effort: Core 0 reads g_hist_data/g_hist_meta
// without a lock while Core 1 keeps appending. A request landing exactly
// as its slot is being recycled can yield a garbled resend, but the
// resent frame carries the same CRC16 as any other frame, so the host
// detects and can simply ask again.
// ===========================================================================
#define HIST_SIZE        16384u   // circular byte storage for frame history
#define HIST_MASK        (HIST_SIZE - 1u)
#define HIST_MAX_ENTRIES 256u     // frame metadata slots, must be power of 2
#define HIST_ENTRY_MASK  (HIST_MAX_ENTRIES - 1u)

typedef struct {
    uint16_t seq_start;
    uint16_t length;
    uint32_t offset;
    bool     valid;
} hist_entry_t;

static uint8_t           g_hist_data[HIST_SIZE];
static hist_entry_t      g_hist_meta[HIST_MAX_ENTRIES];
static volatile uint32_t g_hist_write_off = 0;  // Core 1-owned byte cursor
static volatile uint32_t g_hist_meta_idx  = 0;  // Core 1-owned, monotonic count

/** Core 1 only: append a just-queued frame to the resend history. */
static void hist_append(const uint8_t *frame, uint32_t len, uint16_t seq_start)
{
    uint32_t off = g_hist_write_off;
    for (uint32_t i = 0; i < len; i++)
        g_hist_data[(off + i) & HIST_MASK] = frame[i];
    __sync_synchronize();
    g_hist_write_off = off + len;

    hist_entry_t *e = &g_hist_meta[g_hist_meta_idx & HIST_ENTRY_MASK];
    e->seq_start = seq_start;
    e->length    = (uint16_t)len;
    e->offset    = off;
    __sync_synchronize();
    e->valid = true;
    g_hist_meta_idx++;
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

/** (Re-)apply the ADXL345 operating configuration. Used at startup and to
 *  recover after a periodic DEVID check finds the link out of sync. */
static inline void adxl345_configure(void) {
    write_register(REG_DATA_FORMAT, 0x0Bu);                            // FULL_RES +/-16g
    write_register(REG_BW_RATE,     ODR_3200HZ);                       // 3200 Hz
    write_register(REG_FIFO_CTL,    0x80u | (FIFO_WATERMARK & 0x1Fu)); // STREAM + watermark
    write_register(REG_POWER_CTL,   0x08u);                            // Measurement mode
}

/** Read DEVID and compare to the fixed expected value. The ADXL345 has no
 *  CRC on its SPI output, so this is the practical substitute: DEVID never
 *  changes, so any mismatch means the SPI link (or chip) is not behaving. */
static inline bool adxl345_link_ok(void) {
    uint8_t devid = 0;
    read_registers(REG_DEVID, &devid, 1);
    return devid == DEVID_EXPECTED;
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
//
// Resend protocol: host sends 0xF0 0x0D <seq_lo> <seq_hi> to ask for the
// frame whose header seq_start equals that value. If it's still in
// g_hist_data/g_hist_meta, it's staged into g_resend_buf and drained ahead
// of the live ring on subsequent calls, so the host gets it as soon as
// possible instead of waiting behind whatever is currently streaming.
// ===========================================================================
#define USB_FLUSH_BYTES 256u

#define RESEND_CMD_SYNC1 0xF0u
#define RESEND_CMD_SYNC2 0x0Du

static uint8_t  g_resend_buf[FRAME_BUF_MAX_LEN];
static uint32_t g_resend_len    = 0;
static uint32_t g_resend_off    = 0;
static bool     g_resend_active = false;

/** Core 0 only: look up seq_start in the history and stage it for resend.
 *  Silently does nothing if the frame was never seen or already evicted —
 *  it is the host's job to decide when to give up waiting for a reply. */
static void usb_stage_resend(uint16_t seq)
{
    uint32_t write_off = g_hist_write_off;
    uint32_t count      = g_hist_meta_idx;
    uint32_t n = (count < HIST_MAX_ENTRIES) ? count : HIST_MAX_ENTRIES;

    for (uint32_t i = 0; i < n; i++) {
        hist_entry_t *e = &g_hist_meta[(count - 1u - i) & HIST_ENTRY_MASK];
        if (!e->valid || e->seq_start != seq) continue;
        if (write_off - e->offset > HIST_SIZE) continue;   // already evicted
        uint32_t len = e->length;
        if (len == 0 || len > sizeof(g_resend_buf)) continue;

        for (uint32_t b = 0; b < len; b++)
            g_resend_buf[b] = g_hist_data[(e->offset + b) & HIST_MASK];
        g_resend_len    = len;
        g_resend_off    = 0;
        g_resend_active = true;
        return;
    }
}

/** Core 0 only: parse incoming resend requests from the host. */
static void usb_poll_resend_request(void)
{
    static uint8_t state = 0;   // 0/1 = waiting on sync bytes, 2/3 = seq bytes
    static uint8_t seq_lo = 0;

    uint8_t byte;
    while (tud_cdc_available() && tud_cdc_read(&byte, 1) == 1) {
        switch (state) {
        case 0:
            state = (byte == RESEND_CMD_SYNC1) ? 1u : 0u;
            break;
        case 1:
            state = (byte == RESEND_CMD_SYNC2) ? 2u
                   : (byte == RESEND_CMD_SYNC1) ? 1u : 0u;
            break;
        case 2:
            seq_lo = byte;
            state  = 3u;
            break;
        case 3:
            usb_stage_resend((uint16_t)(seq_lo | ((uint16_t)byte << 8)));
            state = 0u;
            break;
        }
    }
}

/** Core 0 only: drain a staged resend frame, prioritized over live data. */
static void usb_drain_resend(void)
{
    uint32_t remain = g_resend_len - g_resend_off;
    uint32_t avail  = tud_cdc_write_available();
    uint32_t chunk  = (remain < avail) ? remain : avail;
    if (chunk) {
        g_resend_off += tud_cdc_write(&g_resend_buf[g_resend_off], chunk);
    }

    if (g_resend_off >= g_resend_len) {
        g_resend_active = false;
        tud_cdc_write_flush();
    } else if (tud_cdc_write_available() < USB_FLUSH_BYTES) {
        tud_cdc_write_flush();
    }
}

static void usb_service(void)
{
    tud_task();     // TinyUSB event processing (mandatory, Core 0 only)

    g_usb_connected = tud_cdc_connected();
    if (!g_usb_connected) { g_resend_active = false; return; }

    usb_poll_resend_request();

    if (g_resend_active) {
        usb_drain_resend();
        return;     // resend takes priority; live ring resumes next call
    }

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

/** Core 1 only: build and push a status frame reporting the ODR measured
 *  over the last ~1 s window against the RP2040's crystal-timed clock. */
static void push_status_frame(uint32_t odr_mhz)
{
    uint8_t buf[STATUS_FRAME_LEN];
    status_header_t hdr = {
        .sync1   = 0xFA,
        .sync2   = 0xCD,
        .odr_mhz = odr_mhz,
    };
    memcpy(buf, &hdr, sizeof(hdr));
    uint16_t crc = crc16_ccitt(buf, sizeof(hdr));
    buf[sizeof(hdr)]     = (uint8_t)(crc & 0xFFu);
    buf[sizeof(hdr) + 1] = (uint8_t)(crc >> 8u);
    ring_push(buf, sizeof(buf));   // best-effort: a fresh one follows in ~1 s
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
    static uint8_t frame_buf[FRAME_BUF_MAX_LEN];

    uint16_t seq_tx          = 0;
    uint32_t dropped_pending = 0;  // samples dropped since last reported frame
    uint32_t frames_since_check = 0;

    // ODR measurement window: counts samples actually popped from the FIFO
    // against time_us_64() (RP2040 crystal-timed), independent of USB/host
    // jitter. Reset whenever acquisition is paused so a resume doesn't
    // count idle time as part of the window.
    uint64_t odr_window_start_us = time_us_64();
    uint32_t odr_sample_count    = 0;

    while (true) {
        // ---- Pause acquisition while no host is attached ----
        // The ring buffer already holds any not-yet-drained backlog; there is
        // no point polling/packing frames nobody will read. The ADXL345's own
        // 32-entry FIFO keeps overwriting its oldest entry (STREAM mode) until
        // we resume, which is the hardware's natural, bounded fallback.
        if (!g_usb_connected) {
            odr_window_start_us = time_us_64();
            odr_sample_count    = 0;
            sleep_us(LOOP_SLEEP_US);
            continue;
        }

        // ---- Poll FIFO status ----
        uint8_t fifo_status = 0;
        read_registers(REG_FIFO_STATUS, &fifo_status, 1);
        uint8_t entries = fifo_status & 0x3Fu;

        // ---- True overrun comes from INT_SOURCE, not FIFO_STATUS bit 7 ----
        uint8_t int_source = 0;
        read_registers(REG_INT_SOURCE, &int_source, 1);
        uint8_t overrun = (int_source & INT_SOURCE_OVERRUN) ? 1u : 0u;

        if (!overrun && entries < FIFO_WATERMARK) {
            sleep_us(LOOP_SLEEP_US);
            continue;
        }

        // ---- Read samples (6 bytes per call = 1 FIFO sample popped) ----
        uint8_t n = (entries > MAX_SAMPLES_FRAME) ? MAX_SAMPLES_FRAME : entries;
        for (uint8_t i = 0; i < n; i++)
            read_one_xyz6(&raw[6 * i]);
        odr_sample_count += n;

        // ---- Periodic SPI link check (DEVID re-read) ----
        // No per-sample CRC exists on this chip, so instead we periodically
        // confirm the link is still returning the fixed DEVID value. On
        // mismatch, flag it for the host and re-apply the configuration.
        bool link_fault = false;
        if (++frames_since_check >= LINK_CHECK_INTERVAL) {
            frames_since_check = 0;
            if (!adxl345_link_ok()) {
                link_fault = true;
                adxl345_configure();
            }
        }

        // ---- Build frame ----
        // Report drops accumulated from prior iterations; reset only once
        // this report actually makes it into the ring buffer (see below).
        uint16_t dropped_report = (uint16_t)(dropped_pending > 0xFFFFu
                                                  ? 0xFFFFu : dropped_pending);
        frame_header_t hdr = {
            .sync1     = 0xFA,
            .sync2     = 0xCE,
            .seq_start = seq_tx,
            .count     = n,
            .flags     = (uint8_t)(overrun | (dropped_report ? 0x02u : 0x00u)
                                            | (link_fault ? 0x04u : 0x00u)),
            .dropped   = dropped_report,
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
        if (!ring_push(frame_buf, (uint32_t)off)) {
            dropped_pending += n;   // ring full: USB host too slow
        } else {
            dropped_pending = 0;    // report delivered, start a fresh count
            hist_append(frame_buf, (uint32_t)off, hdr.seq_start);
        }

        seq_tx += n;

        // ---- Report measured ODR once the window closes ----
        uint64_t now_us = time_us_64();
        uint64_t elapsed_us = now_us - odr_window_start_us;
        if (elapsed_us >= ODR_REPORT_INTERVAL_US && odr_sample_count > 0) {
            uint32_t odr_mhz = (uint32_t)((uint64_t)odr_sample_count
                                           * 1000000000ull / elapsed_us);
            push_status_frame(odr_mhz);
            odr_window_start_us = now_us;
            odr_sample_count    = 0;
        }

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

    // SPI + GPIO init
    spi_init(SPI_PORT, 5000u * 1000u);  // 5 MHz
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // Power-on settle: call tud_task() every 1 ms for 100 ms.
    // USB enumeration starts immediately after tusb_init() — the host sends
    // SETUP/GET_DESCRIPTOR requests within the first few ms.
    // A plain sleep_ms(100) would leave those requests unanswered and
    // cause enumeration failure (no COM port on host side).
    for (int i = 0; i < 100; i++) {
        tud_task();
        sleep_ms(1);
    }

    // ADXL345 config, with a DEVID read-back to catch a dead/miswired SPI
    // link before we ever start streaming (retry a few times, then proceed
    // anyway — there is no display to halt on, and periodic checks in the
    // sensor loop will keep flagging it to the host).
    for (int attempt = 0; attempt < 3; attempt++) {
        adxl345_configure();
        if (adxl345_link_ok()) break;
    }

    // Launch Core 1 (sensor read; SPI exclusive to Core 1 from here)
    multicore_launch_core1(core1_sensor_loop);

    // Core 0: USB tight loop — no sleep (tud_task needs frequent calls)
    while (true) {
        usb_service();
    }
}
