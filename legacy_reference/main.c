/**
 * Fly-ADXL345-USB High Speed Firmware (FIFO + Batch + CRC + Overrun flag)
 * FIX: ADXL345 cannot output N FIFO samples by reading 6*N bytes in one burst.
 * Must read 6 bytes repeatedly (each 6-byte read pops one FIFO sample).
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/stdio_usb.h"

// ---------------- Pin config ----------------
#define PIN_CS   9
#define PIN_SCK  10
#define PIN_MOSI 11
#define PIN_MISO 12
#define SPI_PORT spi1

// ---------------- ADXL345 registers ----------------
#define REG_BW_RATE       0x2C
#define REG_POWER_CTL     0x2D
#define REG_DATA_FORMAT   0x31
#define REG_DATAX0        0x32
#define REG_FIFO_CTL      0x38
#define REG_FIFO_STATUS   0x39

// ---------------- Stream config ----------------
#define ODR_3200HZ        0x0F
#define FIFO_WATERMARK    16     // 1..31
#define MAX_SAMPLES_FRAME 32     // <= 32
#define LOOP_SLEEP_US     200

#pragma pack(push, 1)
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} sample_xyz_t;

typedef struct {
    uint8_t  sync1;      // 0xFA
    uint8_t  sync2;      // 0xCE
    uint16_t seq_start;  // TX sequence start (LE)
    uint8_t  count;      // number of samples
    uint8_t  flags;      // bit0: fifo_overrun
} frame_header_t;
#pragma pack(pop)

// ---------------- SPI helpers ----------------
static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);
    asm volatile("nop \n nop \n nop");
}
static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

static inline void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect();
}

static inline void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= 0x80;               // READ
    if (len > 1) reg |= 0x40;  // MB
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, buf, len);
    cs_deselect();
}

static inline void read_one_xyz6(uint8_t out6[6]) {
    read_registers(REG_DATAX0, out6, 6);
}

// ---------------- CRC16-CCITT ----------------
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc = (crc << 1);
        }
    }
    return crc;
}

int main() {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);

    // [수정됨] SPI 속도를 500kHz -> 5MHz로 상향
    spi_init(SPI_PORT, 5000 * 1000); 
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    sleep_ms(100);

    // ADXL345 config
    write_register(REG_DATA_FORMAT, 0x0B);    // FULL_RES + ±16g
    write_register(REG_BW_RATE, ODR_3200HZ);  // 3200 Hz
    write_register(REG_FIFO_CTL, (uint8_t)(0x80 | (FIFO_WATERMARK & 0x1F))); // STREAM + watermark
    write_register(REG_POWER_CTL, 0x08);      // Measurement mode

    static uint8_t frame_buf[sizeof(frame_header_t) + MAX_SAMPLES_FRAME * sizeof(sample_xyz_t) + 2];
    uint16_t seq_tx = 0;

    while (true) {
        uint8_t fifo_status = 0;
        read_registers(REG_FIFO_STATUS, &fifo_status, 1);

        uint8_t entries = fifo_status & 0x3F;        // 0..32
        uint8_t overrun = (fifo_status & 0x80) ? 1 : 0;

        if (entries == 0 || entries < FIFO_WATERMARK) {
            sleep_us(LOOP_SLEEP_US);
            continue;
        }

        uint8_t n = entries;
        if (n > MAX_SAMPLES_FRAME) n = MAX_SAMPLES_FRAME;

        uint8_t raw[6 * MAX_SAMPLES_FRAME];
        for (uint8_t i = 0; i < n; i++) {
            read_one_xyz6(&raw[6 * i]);
        }

        frame_header_t hdr;
        hdr.sync1 = 0xFA;
        hdr.sync2 = 0xCE;
        hdr.seq_start = seq_tx;
        hdr.count = n;
        hdr.flags = overrun ? 0x01 : 0x00;

        size_t off = 0;
        memcpy(frame_buf + off, &hdr, sizeof(hdr));
        off += sizeof(hdr);

        sample_xyz_t *samples = (sample_xyz_t *)(frame_buf + off);
        for (uint8_t i = 0; i < n; i++) {
            uint8_t *p = &raw[6 * i];
            samples[i].x = (int16_t)((uint16_t)p[1] << 8 | p[0]);
            samples[i].y = (int16_t)((uint16_t)p[3] << 8 | p[2]);
            samples[i].z = (int16_t)((uint16_t)p[5] << 8 | p[4]);
        }
        off += (size_t)n * sizeof(sample_xyz_t);

        uint16_t crc = crc16_ccitt(frame_buf, off);
        frame_buf[off + 0] = (uint8_t)(crc & 0xFF);
        frame_buf[off + 1] = (uint8_t)((crc >> 8) & 0xFF);
        off += 2;

        fwrite(frame_buf, off, 1, stdout);
        
        // [수정됨] USB 버퍼 비우기 강제 실행 (실시간 전송 보장)
        fflush(stdout); 

        seq_tx = (uint16_t)(seq_tx + n);
        
        if (!overrun) {
            sleep_us(LOOP_SLEEP_US);
        }
    }
}