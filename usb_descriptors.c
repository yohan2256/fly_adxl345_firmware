/**
 * USB descriptors for Fly-ADXL345-USB CDC ACM device
 */
#include <string.h>
#include "tusb.h"

// ------------------------------------------------------------------ IDs ---
#define USBD_VID            0x2E8A   // Raspberry Pi
#define USBD_PID            0x000A   // CDC (matches Pico SDK default CDC PID)
#define USBD_MANUFACTURER   "FlyDrone"
#define USBD_PRODUCT        "ADXL345-USB"
#define USBD_SERIAL         "000001"

// ------------------------------------------------------- Interface numbers --
enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

// ------------------------------------------------------------ Endpoints ---
#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

// ======================================================== Device descriptor
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USBD_VID,
    .idProduct          = USBD_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

// ================================================= Configuration descriptor
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

uint8_t const desc_fs_configuration[] = {
    // Config descriptor (numItf, configNum, iConfig, attrs, power_mA)
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    // CDC descriptor (itfnum, stridx, ep_notif, ep_notif_sz, ep_out, ep_in, ep_sz)
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_fs_configuration;
}

// ====================================================== String descriptors
static char const *string_desc_arr[] = {
    (const char[]){ 0x09, 0x04 },  // 0: Language — English (0x0409)
    USBD_MANUFACTURER,              // 1: Manufacturer
    USBD_PRODUCT,                   // 2: Product
    USBD_SERIAL,                    // 3: Serial number
    "ADXL345 Data",                 // 4: CDC interface name
};

static uint16_t _desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= (uint8_t)(sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
            return NULL;
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = str[i];   // ASCII -> UTF-16LE
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
