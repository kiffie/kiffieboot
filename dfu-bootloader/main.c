/*
 * KB3K DFU boot loader (optimized to fit into 3kB boot FLASH section of PIC32MX2xx)
 *
 */

#include <stdint.h>
#include <xc.h>
#include <usb.h>
#include <usb-ch9.h>

#include <logger.h>

#ifndef LOGLEVEL_MAIN
#define LOGLEVEL_MAIN LOG_VERBOSE
#endif
#define LOGLEVEL   LOGLEVEL_MAIN
#define LOG_PREFIX "MAIN"

#define USBEP_SIZE 64

#if __PIC32_FEATURE_SET == 250 || __PIC32_FEATURE_SET == 270
#define FLASH_ROWSIZE  128
#define FLASH_PAGESIZE 1024
#elif __PIC32_FEATURE_SET == 470 || __PIC32_FEATURE_SET == 274
#define FLASH_ROWSIZE  512
#define FLASH_PAGESIZE 4096
#else
#error unsupported device
#endif
#define FLASH_ROWS_PER_PAGE (FLASH_PAGESIZE / FLASH_ROWSIZE)
#define FLASH_BASE          0x1d000000
#define FLASH_SIZE          (__PIC32_MEMORY_SIZE * 1024)

#define NVMOP_WORD_PGM   0x1
#define NVMOP_PAGE_ERASE 0x4
#define NVMOP_PFM_ERASE  0x5
#define NVMOP_ROW_PGM    0x3
#define NVMOP_NOP        0x0

#ifndef SYS_CLOCK
#error "Macro SYS_CLOCK not defined (must be system clock im Hz)"
#endif
#define TIMER_TICKS_PER_US     (SYS_CLOCK / 2000000)
#define TIMER_TICKS_PER_MS     (TIMER_TICKS_PER_US * 1000)
#define TIMER_TICKS_PER_SECOND (TIMER_TICKS_PER_US * 1000000)

#define MIN(a, b) (a < b ? a : b)
#define MAX(a, b) (a > b ? a : b)

/* helper macros for encoding values > 255 */
#define BYTE0(word) (word & 0xff)
#define BYTE1(word) ((word >> 8) & 0xff)

/*
 * DFU
 */
enum dfu_request_value {
    DFU_DETACH = 0,
    DFU_DNLOAD = 1,
    DFU_UPLOAD = 2,
    DFU_GETSTATUS = 3,
    DFU_CLRSTATUS = 4,
    DFU_GETSTATE = 5,
    DFU_ABORT = 6,
};

typedef struct __attribute__((__packed__)) dfu_status {
    uint8_t bStatus;
    uint8_t bwPollTimeout[3];
    uint8_t bState;
    uint8_t iString;
} dfu_status_t;

enum dfu_status_value {
    DFU_STATUS_OK = 0x00,
    DFU_STATUS_ERR_WRITE = 0x03,
    DFU_STATUS_ERR_ERASE = 0x04,
    DFU_STATUS_ERR_STALLEDPKT = 0x0f,
};

enum dfu_state {
    DFU_STATE_IDLE = 2,
    DFU_STATE_DNLOAD_SYNC = 3,
    DFU_STATE_BUSY = 4,
    DFU_STATE_DNLOAD_IDLE = 5,
    DFU_STATE_MANIFEST_SYNC = 6,
    DFU_STATE_MANIFEST = 7,
    DFU_STATE_MANIFEST_WAIT_RESET = 8,
    DFU_STATE_UPLOAD_IDLE = 9,
    DFU_STATE_ERROR = 10,
};

/*
 * USB Descriptors
 */
struct __attribute__((__packed__)) dfu_functional_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bmAttributes;
    uint16_t wDetachTimeOut;
    uint16_t wTransferSize;
    uint16_t bcdDFUVersion;
};

static const struct usb9_device_descriptor usb_device_desc = {
    .bLength = 18,
    .bDescriptorType = USB9_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x16c0,  /* see USB-IDs-for-free.txt */
    .idProduct = 0x05dc, /* PID for Vendor Class devices with libusb */
    .bcdDevice = 0,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 0,
    .bNumConfigurations = 1};

struct __attribute__((__packed__)) my_config_descriptor {
    struct usb9_configuration_descriptor config_desc;
    struct usb9_interface_descriptor interface_desc;
    struct dfu_functional_descriptor dfu_func_desc;
};

static const struct my_config_descriptor usb_config_desc = {
    .config_desc = {
        .bLength = sizeof(struct usb9_configuration_descriptor),
        .bDescriptorType = USB9_DESC_CONFIGURATION,
        .wTotalLength = sizeof(struct my_config_descriptor),
        .bNumInterfaces = 1,
        .bConfigurationValue = 42,
        .iConfiguration = 0,
        .bmAttributes = 0x80,
        .bMaxPower = 100 /* 200 mA */
    },
    .interface_desc = {
        .bLength = sizeof(struct usb9_interface_descriptor),
        .bDescriptorType = USB9_DESC_INTERFACE,
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = 0,
        .bInterfaceClass = 0xfe, /* DFU interface class */
        .bInterfaceSubClass = 1, /* DFU firmware upgrade */
        .bInterfaceProtocol = 2, /* DFU mode protocol */
        .iInterface = 0
    },
    .dfu_func_desc = {
        .bLength = 9,
        .bDescriptorType = 0x21, /* DFU functional descriptor type */
        .bmAttributes = 0b0001,
        .wDetachTimeOut = 60000,
        .wTransferSize = FLASH_PAGESIZE,
        .bcdDFUVersion = 0x0110,
    }
};

static const uint16_t usb_string_desc_0[] = {
    USB9_DESC_STRING_HEADER(1),
    USB9_LANGID_EN_US /* LANGID code 0 */
};

static const uint16_t usb_string_desc_1[] = {
    USB9_DESC_STRING_HEADER(18),
    'k', 'i', 'f', 'f', 'i', 'e', '@',
    'm', 'a', 'i', 'l', 'b', 'o', 'x', '.', 'o', 'r', 'g'
};

static const uint16_t usb_string_desc_2[] = {
    USB9_DESC_STRING_HEADER(4),
    'k', 'b', '3', 'k'
};

static const usb_desc_table_elem_t usb_desc_table[] = {
    {0, USB9_DESC_DEVICE, 0, &usb_device_desc, sizeof(usb_device_desc)},
    {0, USB9_DESC_CONFIGURATION, 0, &usb_config_desc, sizeof(usb_config_desc)},
    {0, USB9_DESC_STRING, 0, usb_string_desc_0, sizeof(usb_string_desc_0)},
    {1, USB9_DESC_STRING, USB9_LANGID_EN_US, usb_string_desc_1, sizeof(usb_string_desc_1)},
    {2, USB9_DESC_STRING, USB9_LANGID_EN_US, usb_string_desc_2, sizeof(usb_string_desc_2)},
    {0, 0, 0, NULL, 0}};

typedef uint32_t dma_addr_t;

static struct {
    /* make row buffer large enough to enable transfer with size < FLASH_PAGESIZE and successive
     * transfer with size FLASH_PAGESIZE */
    uint8_t page_buffer[2 * FLASH_PAGESIZE];
    size_t page_buffer_ndx;
    uint8_t dfu_state;
    uint8_t dfu_status;
    uint32_t startup_timeout;
    size_t flash_offset; /* current offset within program flash in bytes */
} bl;

/* This is the only persistent variable, so it must be at 0xa0000000 */
uint32_t __attribute__((persistent, section(".pbss"))) boot_magic;

#define BOOT_MAGIC 0x746f6f62

inline dma_addr_t __attribute__((always_inline)) virt_to_phys(const void *p) {
    return (int)p < 0 ? ((int)p & 0x1fffffffL) : (unsigned int)((unsigned char *)p + 0x40000000L);
}

static uint32_t __attribute__((nomips16)) get_mips_count(void) {
    uint32_t count;
    asm volatile("mfc0   %0, $9" : "=r"(count));
    return count;
}

static void __attribute__((nomips16)) set_mips_count(uint32_t count){
    asm volatile("mtc0   %0, $9" : : "r"(count));
}

static bool is_app_present(void) {
    /* if first word of program memory is erased, we assume that the app is not
     * present */
    return *((uint32_t *)0x9d000000) != 0xffffffff;
}

void start_app(void) {
    uint32_t rcon = RCON; /* save content of RCON before modifying it */
    RCONCLR = 0xff;
    NVMCONCLR = _NVMCON_WREN_MASK;
    boot_magic = 0; /* clear magic number */
    void (*app)(uint32_t);
    app = (void (*)(uint32_t))0x9d000000;
    app(rcon); /* pass original content of RCON to app */
}

static unsigned nvm_op(unsigned op) {
    log_debug("nvm_op: op = %x, NVMSRCADDR = %08x, NVMADDR = %08x\n", op, NVMSRCADDR, NVMADDR);
    NVMCON = _NVMCON_WREN_MASK | op;
    NVMKEY = 0xAA996655;
    NVMKEY = 0x556699AA;
    NVMCONSET = _NVMCON_WR_MASK;
    while (NVMCON & _NVMCON_WR_MASK);
    NVMCONCLR = _NVMCON_WREN_MASK;
    return NVMCON & (_NVMCON_WRERR_MASK | _NVMCON_LVDERR_MASK);
}

static bool boot_switch_active(void) {
    /* start bootloader in case of external reset triggered via the reset pin */
    return !(RCON & _RCON_POR_MASK);
}

static int setup_handler(usb9_setup_data_t *setup_packet, void **inout_data) {

    int len = -1;

    /* program a full flash page */
    if (bl.page_buffer_ndx >= FLASH_PAGESIZE) {
        log_debug("Flashing page @%08lx, page_buffer_ndx = %lu\n", FLASH_BASE + bl.flash_offset, bl.page_buffer_ndx);
        log_debug("start: "); log_debug_hexdump(bl.page_buffer, 16);
        NVMADDR = FLASH_BASE + bl.flash_offset;
        unsigned nvm_stat = nvm_op(NVMOP_PAGE_ERASE);
        if (nvm_stat != 0) {
            log_debug("Flash page erase failed: %04x\n", nvm_stat);
            bl.dfu_state = DFU_STATE_ERROR;
            bl.dfu_status = DFU_STATUS_ERR_ERASE;
            goto error;
        }
        for (size_t i = 0; i < FLASH_ROWS_PER_PAGE; i++) {
            NVMADDR = FLASH_BASE + bl.flash_offset;
            NVMSRCADDR = virt_to_phys(&bl.page_buffer[0]) + i * FLASH_ROWSIZE;
            nvm_stat = nvm_op(NVMOP_ROW_PGM);
            if (nvm_stat != 0) {
                log_debug("Flash row program failed: %04x\n", nvm_stat);
                bl.dfu_state = DFU_STATE_ERROR;
                bl.dfu_status = DFU_STATUS_ERR_WRITE;
                goto error;
            }
            bl.flash_offset += FLASH_ROWSIZE;
        }
        /* remove programmed page from page buffer */
        bl.page_buffer_ndx -= FLASH_PAGESIZE;
        memmove(bl.page_buffer, &bl.page_buffer[FLASH_PAGESIZE], bl.page_buffer_ndx);
    }

    switch (setup_packet->bRequest) {
        case DFU_GETSTATUS: {
            log_debug("DFU_GETSTATUS\n");
#ifdef BL_TIMEOUT
            bl.startup_timeout = 0;
#endif
            static dfu_status_t status;
            memset(&status, 0, sizeof(status));
            status.bStatus = bl.dfu_status;
            status.bState = bl.dfu_state;
            *inout_data = &status;
            len = sizeof(status);
            if (bl.dfu_state == DFU_STATE_MANIFEST_SYNC) {
                bl.dfu_state = DFU_STATE_MANIFEST_WAIT_RESET;
            }
            break;
        }
        case DFU_DNLOAD: {
            log_debug("DFU_DNLOAD block %u\n", setup_packet->wValue);
            if (setup_packet->wLength > FLASH_PAGESIZE) {
                log_debug("invalid transfer size wLength = %u\n", setup_packet->wLength);
                break;
            }
            *inout_data = &bl.page_buffer[bl.page_buffer_ndx];
            len = setup_packet->wLength;
            bl.page_buffer_ndx += len;
            if (len > 0) {
                bl.dfu_state = DFU_STATE_DNLOAD_IDLE;
            } else if (bl.page_buffer_ndx > 0) { /* download complete */
                /* pad unused part of page buffer with 0xff and extend page puffer to FLASH_PAGESIZE */
                memset(&bl.page_buffer[bl.page_buffer_ndx], 0xff, sizeof(bl.page_buffer) - bl.page_buffer_ndx);
                bl.page_buffer_ndx = FLASH_PAGESIZE;
                bl.dfu_state = DFU_STATE_MANIFEST_SYNC;
            }
            break;
        }
    }
    log_debug("len = %d\n", len);
    if (len < 0) {
        bl.dfu_status = DFU_STATUS_ERR_STALLEDPKT;
        bl.dfu_state = DFU_STATE_ERROR;
    }
error:
    return len;
}

static void control_xfer_complete(bool canceled) {
    /* nothing to do */
}

void usb_reset_hook(void) {
    if (bl.dfu_state > DFU_STATE_IDLE) {
        log_debug("starting application after USB reset\n");
        usb_shutdown();
        start_app();
    }
}

int main(int argc, char **argv) {

#if LOGLEVEL > LOG_NONE
#if __PIC32_FEATURE_SET == 470
    RPF5R = 0b0001; /* U2TX */
#else
    RPB0R = 0b0010; /* U2TX */
#endif
    term_init();
    term_write_char('\n');
#endif
    boot_switch_active(); /* call this to initialize GPIO port */
    memset(&bl, 0, sizeof(bl));
    log_info("=== DFU Bootloader ===\n");
    log_debug("Reset Control Register: RCON = 0x%x\n", RCON);
    log_debug("row_buffer = %p\n", bl.page_buffer);
    log_debug("FLASH_ROWSIZE = %u\n", FLASH_ROWSIZE);
    log_debug("boot_magic = %08x\n", (unsigned)boot_magic);

    bool app_present = is_app_present();
    log_debug("Application present: %u\n", app_present);
    bool sw_reset = RCON & _RCON_SWR_MASK;
    log_debug("SW reset: %u\n", sw_reset);

    if (!(sw_reset && boot_magic == BOOT_MAGIC) &&
        !boot_switch_active() &&
        app_present)
    {
        log_debug("starting application\n");
        start_app();
    }
    log_debug("starting bootloader\n");
    bl.dfu_state = DFU_STATE_IDLE;
    usb_init_with_desc(usb_desc_table);
    usb_set_nonstd_req_handler(setup_handler, control_xfer_complete);
    log_debug("USB initialized\n");
#ifdef BL_TIMEOUT
    set_mips_count(0);
    if (app_present) {
        bl.startup_timeout = BL_TIMEOUT * TIMER_TICKS_PER_SECOND;
    }
#endif
    while (1) {
        if (bl.startup_timeout != 0 && get_mips_count() >= bl.startup_timeout) {
            log_info("timeout -> starting application\n");
            usb_shutdown();
            start_app();
        }
        usb_tasks();
    }
    return 0;
}
