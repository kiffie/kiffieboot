/*
 * simple USB driver (device mode) for PIC32MX
 *
 * Copyright (C) 2019, 2020 Stephan <kiffie@mailbox.org>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#include <sys/attribs.h>
#include <xc.h>
#include <usb.h>
#include <usb-ch9.h>
#include <string.h>

#include <logger.h>

#ifndef LOGLEVEL_USB
#define LOGLEVEL_USB LOG_INFO
#endif
#define LOGLEVEL LOGLEVEL_USB
#define LOG_PREFIX "USB"

#define USBERR -1 /* return value to indicate an error */

#define MIN(a, b) (a < b ? a : b)
#define MAX(a, b) (a > b ? a : b)

/*
 * state/context information
 */

#define USB_BD_UOWN   0x80
#define USB_BD_DATA01 0x40
#define USB_BD_KEEP   0x20
#define USB_BD_NINC   0x10
#define USB_BD_DTS    0x08
#define USB_BD_STALL  0x04

#define USB_BD_PID_POS 2
#define USB_BD_PID_MSK 0x3c
#define USB_BD_PID_LEN 4

typedef size_t dma_addr_t;

typedef struct usb_buffer_descriptor {
    uint16_t flags;
    uint16_t byte_count;
    dma_addr_t buffer_address; /* physical address */
} usb_buffer_descriptor_t;

static union {
    usb_buffer_descriptor_t flat[4 * USB_NO_ENDPOINTS];
    usb_buffer_descriptor_t ep_dir_ppbi[USB_NO_ENDPOINTS][2][2];
} usb_bdt __attribute__((aligned(512)));

typedef struct usb_ep_info {

    bool next_odd; /* next BD to use is odd BD */
    bool data01;   /* data toggle flag for next transaction */
    bool stalled;

    unsigned ep_size;     /* max. transaction size */
    uint8_t bmAttributes; /* currently used to store EP type */
    void *ep_buf[2];      /* virtual addresses of transaction buffers */

    usb_completion_cb_t ep_handler; /* pointer to handler function */
} usb_ep_info_t;

enum usb_control_state {
    USB_CS_IDLE,  /* normal data transfers */
    USB_CS_WDATA, /* control write, data stage */
    USB_CS_WSTAT, /* control write, status stage */
    USB_CS_RDATA, /* control read, data stage */
    USB_CS_RZLP,  /* control read, armed for zero length packet */
    USB_CS_RSTAT, /* control read, status stage */
    USB_CS_STATUS /* status stage without preceding data stage */
};

typedef struct usb_context {
    const usb_desc_table_elem_t *desc_table;
    usb_ep_info_t ep_info[USB_NO_ENDPOINTS][2]; /* 0: OUT (RX); 1: IN (TX) */

    /* endpoint zero */
    enum usb_control_state ep0_state;
    uint8_t ep0_outbuf[USB_EP0_SIZE]; /* buffer for SETUP and OUT transactions */
    usb9_setup_data_t ep0_sudata;
    uint8_t *ep0_xfer_buf; /* buffer for control data stage */
    unsigned ep0_xfer_len;
    unsigned ep0_xfer_ndx;

    uint8_t ep0_new_address;
#ifndef USB_DISABLE_NONSTD_CTRL
    usb_crtl_setup_cb_t ep0_nonstd_request_handler;
    usb_ctrl_data_complete_cb_t ep0_nonstd_data_complete;
#endif
} usb_context_t;

static usb_context_t usb;

inline dma_addr_t __attribute__((always_inline)) usb_virt_to_phys(const void *p) {
    return (int)p < 0 ? ((int)p & 0x1fffffffL) : (unsigned int)((unsigned char *)p + 0x40000000L);
}

static void usb_ep0_handler(unsigned ep, unsigned pid, void *buffer, unsigned len);

#ifdef USB_DISABLE_IRQ
static inline void usb_ei(void) {}
static inline void usb_di(void) {}
#else
static inline void usb_ei(void) { IEC1SET = _IEC1_USBIE_MASK; }
static inline void usb_di(void) { IEC1CLR = _IEC1_USBIE_MASK; }
#endif

#ifdef USB_DISABLE_IRQ
void usb_tasks(void)
#elif defined(USB_INDIRECT_ISR)
void usb_isr(void)
#else
void __ISR(_USB_1_VECTOR, IPL4SOFT) usb_irq(void)
#endif
{
    log_verbose("------ IRQ ------\n");
    while (U1IRbits.TRNIF) {
        int u1stat = U1STAT; /* copy status to CPU register */
        U1IR = _U1IR_TRNIF_MASK;
        unsigned ep = (u1stat & _U1STAT_ENDPT_MASK) >> _U1STAT_ENDPT_POSITION;
        unsigned dir = (u1stat & _U1STAT_DIR_MASK) >> _U1STAT_DIR_POSITION;
        struct usb_ep_info *ep_info = &usb.ep_info[ep][dir];
        unsigned bdt_index = u1stat >> 2;
        struct usb_buffer_descriptor *bd = &usb_bdt.flat[bdt_index];
        int pid = (bd->flags & USB_BD_PID_MSK) >> USB_BD_PID_POS;
        bool data01 = bd->flags & USB_BD_DATA01;
        log_verbose("!TRNIF U1STAT=%02x ep=%d, pid=%d, was_odd=%d, data01=%d, byte_count=%u\n",
                    u1stat, ep, pid, u1stat & _U1STAT_PPBI_MASK ? 1 : 0, data01, bd->byte_count);
        if (ep_info->ep_handler != NULL) {
            uint8_t *buf = ep_info->ep_buf[u1stat & _U1STAT_PPBI_MASK ? 1 : 0];
            ep_info->ep_handler(ep, pid, buf, bd->byte_count);
        }
    }
    if (U1IRbits.URSTIF) {
        log_verbose("!USB RESET IRQ, U1CON=%02x, U1ADDR=%02x, U1EP0=%02x\n",
                    U1CON, U1ADDR, U1EP0);
        U1ADDR = 0;
        U1IR = _U1IR_URSTIF_MASK;
#ifdef USB_RESET_HOOK
        usb_reset_hook();
#endif
    }
    if (U1IRbits.STALLIF) {
        log_verbose("!STALL IRQ\n");
        U1EP0bits.EPSTALL = 0;
        //usb_arm_rx(0, 0);
        U1IR = _U1IR_STALLIF_MASK;
    }
    IFS1CLR = _IFS1_USBIF_MASK;
}

void usb_init_endpoint(uint8_t ep_addr,
                       uint8_t bmAttributes,
                       unsigned ep_size,
                       usb_completion_cb_t handler) {
    unsigned is_in = ep_addr & 0x80 ? 1 : 0;
    unsigned ep = ep_addr & 0x0f;
    struct usb_ep_info *info = &usb.ep_info[ep][is_in];
    memset(info, 0, sizeof(struct usb_ep_info));
    info->ep_size = ep_size;
    info->bmAttributes = bmAttributes;
    info->ep_handler = handler;

    usb_buffer_descriptor_t *bd = usb_bdt.ep_dir_ppbi[ep][0];
    memset(bd, 0, 2 * sizeof(usb_buffer_descriptor_t));
    volatile unsigned *u1ep_reg = &U1EP0 + 4 * ep; /* address of EP control reg */
    if (is_in) {
        *u1ep_reg |= _U1EP0_EPTXEN_MASK;
    } else {
        *u1ep_reg |= _U1EP0_EPRXEN_MASK;
    }
    if ((bmAttributes & USB9_EPDESC_ATTR_TYPE_MASK) != USB9_EPDESC_ATTR_TYPE_ISO) {
        *u1ep_reg |= _U1EP0_EPHSHK_MASK;
    }
    if ((bmAttributes & USB9_EPDESC_ATTR_TYPE_MASK) != USB9_EPDESC_ATTR_TYPE_CONTROL) {
        *u1ep_reg |= _U1EP0_EPCONDIS_MASK;
    }
#if LOGLEVEL >= LOG_DEBUG
    static char DIR[2][4] = {"OUT", "IN "};
    log_debug("initialized EP%d %s ep_size=%d, u1epX@%p=%02x\n",
              ep_addr & 0xf, DIR[ep_addr >> 7], ep_size, u1ep_reg, *u1ep_reg);
#endif
}

static usb_buffer_descriptor_t *get_buffer_desc(unsigned ep, bool is_in) {
    bool ppbi = usb.ep_info[ep][is_in].next_odd;
    return &usb_bdt.ep_dir_ppbi[ep][is_in][ppbi];
}

static int usb_arm_generic(int ep, bool is_in, void *data, unsigned data_len, bool stall) {

    struct usb_ep_info *ep_info = &usb.ep_info[ep][is_in];
    volatile struct usb_buffer_descriptor *bd = get_buffer_desc(ep, is_in);

    if (ep_info->stalled) {
        ep_info->stalled = 0;
        bd->flags = 0;
        log_debug("was stalled\n");
    }
    if (bd->flags & USB_BD_UOWN) {
        log_error("buffer descriptor not free, ep=%d, dir=%d\n", ep, is_in);
        return USBERR;
    }
    if (data_len > ep_info->ep_size) {
        log_error("data_len > ep_size=%d", ep_info->ep_size);
        return USBERR;
    }
    bool is_iso =
        (ep_info->bmAttributes & USB9_EPDESC_ATTR_TYPE_MASK) == USB9_EPDESC_ATTR_TYPE_ISO;
    ep_info->ep_buf[ep_info->next_odd] = data;
    bd->buffer_address = usb_virt_to_phys(data);
    bd->byte_count = data_len;
    bd->flags = USB_BD_UOWN | (ep_info->data01 ? USB_BD_DATA01 : 0) | (is_iso ? 0 : USB_BD_DTS) | (stall ? USB_BD_STALL : 0);
    log_debug("armed: ep=%d, is_in=%u, is_odd=%u, data01=%u, data_len=%d, flags=%02x\n",
              ep, is_in, ep_info->next_odd,
              ep_info->data01, data_len, bd->flags);
    ep_info->next_odd = !ep_info->next_odd;
    if (!is_iso) {
        ep_info->data01 = !ep_info->data01;
    }
    return 0;
}

/**
 * Arm an endpoint.
 * @returns returns true on success.
 */
bool usb_arm_endpoint(uint8_t ep_addr, void *data, unsigned data_len) {
    int r = usb_arm_generic(ep_addr & 0x0f, ep_addr & 0x80, data, data_len, false);
    return r == 0;
}

/* cancel transactions without calling the callback functions */
static void usb_cancel_transactions(uint8_t ep_addr) {
    size_t ep = ep_addr & 0x0f;
    bool is_in = ep_addr & 0x80;
    struct usb_ep_info *ep_info = &usb.ep_info[ep][is_in];
    if (usb_bdt.ep_dir_ppbi[ep][is_in][0].flags & USB_BD_UOWN &&
       !(usb_bdt.ep_dir_ppbi[ep][is_in][0].flags & USB_BD_STALL))
    {
        ep_info->next_odd = !ep_info->next_odd;
        usb_bdt.ep_dir_ppbi[ep][is_in][0].flags = 0;
    }
    if (usb_bdt.ep_dir_ppbi[ep][is_in][1].flags & USB_BD_UOWN &&
       !(usb_bdt.ep_dir_ppbi[ep][is_in][1].flags & USB_BD_STALL))
    {
        ep_info->next_odd = !ep_info->next_odd;
        usb_bdt.ep_dir_ppbi[ep][is_in][1].flags = 0;
    }
}

static int usb_arm_ep0out(void) {
    return usb_arm_generic(0, 0, usb.ep0_outbuf, sizeof(usb.ep0_outbuf), false);
}

static int usb_arm_ep0in(void *data, int data_len) {
    return usb_arm_generic(0, 1, data, data_len, false);
}

static void usb_set_data01(int ep, bool is_in, bool data01) {
    usb.ep_info[ep][is_in].data01 = data01;
}

void usb_stall_endpoint(int ep, bool is_in) {

    usb_ep_info_t *ep_info = &usb.ep_info[ep][is_in];
    volatile usb_buffer_descriptor_t *bd = get_buffer_desc(ep, is_in);

    if (ep_info->stalled) {
        log_debug("EP%d: is_in=%u, odd=%u already stalled\n",
                  ep, is_in, ep_info->next_odd);
    } else {
        bd->flags = USB_BD_STALL | USB_BD_UOWN;
        ep_info->stalled = 1;
        log_debug("stalling EP%d: is_in=%u, next_odd=%u\n",
                  ep, is_in, ep_info->next_odd);
        //ep_info->rx_next_odd= ! ep_info->rx_next_odd;
    }
}

static void usb_stall_ep0(void) {
    U1EP0SET = _U1EP0_EPSTALL_MASK;
}

/**
 * handler for EP0 standard control requests.
 *
 * inout_data is a pointer to a pointer to a data stage buffer. The handler
 * must provide the data stage buffer and set *inout_data. For OUT transfers,
 * the length of the buffer must be at least wLength.
 *
 * returns: the length of the data to be returned to the host for IN transfers
 *          and wLength for OUT transfers,
 *          zero if no data should be returned (no data stage)
 *          -1 in case of a Request Error (indicates that EP0 shall be stalled
 *          without entering the data phase)
 */
static int usb_ep0_std_request_cb(usb9_setup_data_t *setup_data,
                                  const uint8_t **inout_data) {
    int res = -1;
    static uint16_t word_buf;

    switch (setup_data->bRequest) {
        case USB9_REQ_GET_STATUS:
            log_debug("get status\n");
            word_buf = 0;
            *inout_data = (uint8_t *)&word_buf;
            res = 2;
            break;

        case USB9_REQ_GET_DESCRIPTOR:;
            uint8_t desc_index = setup_data->wValue;
            uint8_t desc_type = setup_data->wValue >> 8;
            const usb_desc_table_elem_t *desc_elem = usb.desc_table;
            while (desc_elem->desc != NULL) {
                if (desc_elem->desc_index == desc_index &&
                    desc_elem->desc_type == desc_type &&
                    desc_elem->wIndex == setup_data->wIndex) {
                    break;
                }
                desc_elem++;
            }
            if (desc_elem->desc != NULL) {
                log_info("get descriptor, wValue = 0x%04x, wLength = %u, desc_len = %u\n",
                         setup_data->wValue, setup_data->wLength, desc_elem->desc_len);
                *inout_data = desc_elem->desc;
                res = MIN(desc_elem->desc_len, setup_data->wLength);
            } else { /* stall on unsupported descriptor type */
                log_error("get request for unsupported descriptor type 0x%04x\n",
                          setup_data->wValue);
                res = -1;
            }
            break;
        case USB9_REQ_SET_ADDRESS:
            log_debug("set address request: addr=%d\n", setup_data->wValue);
            usb.ep0_new_address = setup_data->wValue;
            res = 0; /* status stage */
            break;
        case USB9_REQ_SET_CONFIGURATION:
            log_debug("set configuration request: config=%d\n", setup_data->wValue);
            res = 0;
            break;
        case USB9_REQ_SET_INTERFACE:
            log_debug("set interface request: bAltSettig=%d, wInterface=%d\n",
                      setup_data->wValue, setup_data->wIndex);
            res = 0;
            break;
    }
    return res;
}

#ifndef USB_DISABLE_NONSTD_CTRL
static int usb_default_ep0_request_cb(usb9_setup_data_t *setup_packet, void **inout_data) {
    return -1;
}
#endif

static void call_data_complete_handler(bool canceled) {
#ifndef USB_DISABLE_NONSTD_CTRL
    if (!usb9_is_std_request(usb.ep0_sudata.bmRequestType) &&
       usb.ep0_nonstd_data_complete != NULL)
    {
        usb.ep0_nonstd_data_complete(canceled);
    }
#endif
}

/* to be called in states USB_CS_IDLE or USB_CS_RDATA after IN transaction */
static enum usb_control_state usb_ep0_arm_for_read_data_stage(void) {
    enum usb_control_state next_state = USB_CS_RDATA;
    unsigned txlen = MIN(usb.ep0_xfer_len - usb.ep0_xfer_ndx, USB_EP0_SIZE);
    if (txlen > 0) {
        usb_arm_generic(0, true,
                        &usb.ep0_xfer_buf[usb.ep0_xfer_ndx], txlen,
                        false);
        usb.ep0_xfer_ndx += txlen;
    } else { /* all data sent -> transition to next state */
        /* Send an empty IN packet if the transfer is truncated and
         * the length of the last packet of the transfer corresponds to the
         * endpoint size or if the transfer is truncated to zero length.
         */
        if (usb.ep0_sudata.wLength > usb.ep0_xfer_len &&
           (usb.ep0_xfer_len % USB_EP0_SIZE) == 0)
        {
            usb_arm_generic(0, true, NULL, 0, false);
            next_state = USB_CS_RZLP;
        } else {
            next_state = USB_CS_RSTAT;
        }
        call_data_complete_handler(false);
    }
    return next_state;
}

static void usb_ep0_handler(unsigned ep, unsigned pid, void *buffer, unsigned len) {

    switch (pid) {
        case USB_PID_SETUP:
            if (usb.ep0_state != USB_CS_IDLE) {
                log_debug("unexpected SETUP token, state = %u\n",
                          usb.ep0_state);
                /* call handler if there are outstanding data phases */
                if (usb.ep0_state == USB_CS_WDATA ||
                    usb.ep0_state == USB_CS_RDATA)
                {
                     call_data_complete_handler(true);
                }
                usb.ep0_state = USB_CS_IDLE;
            }
            usb_cancel_transactions(0x80); /* cancel IN transactions */
            U1CONbits.PKTDIS = 0;
            memcpy(&usb.ep0_sudata, usb.ep0_outbuf, sizeof(usb.ep0_sudata));
            log_info("SETUP bmRequestType=%02x, bRequest=%02x, "
                     "wValue=%04x, wIndex=%u, wLength=%u\n",
                     usb.ep0_sudata.bmRequestType,
                     usb.ep0_sudata.bRequest,
                     usb.ep0_sudata.wValue,
                     usb.ep0_sudata.wIndex,
                     usb.ep0_sudata.wLength);
            int xfer_len;
            if (usb9_is_std_request(usb.ep0_sudata.bmRequestType)) {
                xfer_len = usb_ep0_std_request_cb(
                    &usb.ep0_sudata,
                    (const uint8_t **)&usb.ep0_xfer_buf);
            } else {
#ifndef USB_DISABLE_NONSTD_CTRL
                xfer_len = usb.ep0_nonstd_request_handler(
                    &usb.ep0_sudata,
                    (void **)&usb.ep0_xfer_buf);
#else
                xfer_len = -1;
#endif
            }
            bool stall = xfer_len < 0 || xfer_len > usb.ep0_sudata.wLength;
            if (usb.ep0_sudata.wLength > 0) { /* DATA phase present */
                usb.ep0_xfer_len = xfer_len;
                usb.ep0_xfer_ndx = 0;
                bool is_in = usb.ep0_sudata.bmRequestType & USB9_RT_IN;
                if (!stall) {
                    usb_set_data01(0, false, 1);
                    usb_set_data01(0, true, 1);
                    if (is_in) { /* control read transfer */
                        usb.ep0_state = usb_ep0_arm_for_read_data_stage();
                    } else { /* control write transfer */
                        usb.ep0_state = USB_CS_WDATA;
                    }
                }
            } else { /* control transfer without data phase */
                usb.ep0_state = USB_CS_STATUS;
                if (!stall) {
                    usb_set_data01(0, 1, 1);
                    usb_arm_ep0in(NULL, 0);
                }
            }
            if (stall) {
                log_info("invalid request: bRequest = %d, xfer_len=%d, wLength=%u\n",
                         usb.ep0_sudata.bRequest, xfer_len, usb.ep0_sudata.wLength);
                usb.ep0_state = USB_CS_IDLE;
                usb_stall_ep0();
                if (xfer_len > 0)
                    call_data_complete_handler(true);
            }
            usb_arm_ep0out();
            break;

        case USB_PID_IN:
            log_debug("IN (=TX) complete on EP0, len=%d\n", len);
            switch (usb.ep0_state) {
                case USB_CS_RDATA:
                    usb.ep0_state = usb_ep0_arm_for_read_data_stage();
                    break;

                case USB_CS_RZLP:
                    usb.ep0_state = USB_CS_RSTAT;
                    break;

                case USB_CS_WSTAT:
                    log_debug("control write complete\n");
                    usb.ep0_state = USB_CS_IDLE;
                    break;

                case USB_CS_STATUS:
                    usb.ep0_state = USB_CS_IDLE;
                    log_debug("control transfer (w/o data stage) complete.\n");
                    if (usb.ep0_new_address > 0) {
                        U1ADDR = _U1ADDR_DEVADDR_MASK & usb.ep0_new_address;
                        usb.ep0_new_address = 0;
                        log_debug("set address to %d\n", U1ADDR);
                    }
                    break;
                default:
                    log_error("IN: unexpected event\n");
                    usb_stall_ep0();
            }
            break;

        case USB_PID_OUT:
            log_debug("OUT (=RX) complete on EP0, len=%d\n", len);
            switch (usb.ep0_state) {
                case USB_CS_WDATA:;
                    /* copy data from buffer and increase xfer_ndx */
                    size_t txlen = MIN(usb.ep0_xfer_len - usb.ep0_xfer_ndx,
                                       sizeof(usb.ep0_outbuf));
                    memcpy(&usb.ep0_xfer_buf[usb.ep0_xfer_ndx],
                           usb.ep0_outbuf,
                           sizeof(usb.ep0_outbuf));
                    usb.ep0_xfer_ndx += txlen;
                    if (usb.ep0_xfer_ndx >= usb.ep0_xfer_len) { /* transfer complete */
                        usb.ep0_state = USB_CS_WSTAT;
                        usb_arm_ep0in(NULL, 0); /* for status phase */
                        call_data_complete_handler(false);
                    }
                    break;

                case USB_CS_RDATA:
                    log_debug("early termination of read\n");
                    call_data_complete_handler(true);
                    /* intentionally no break here */
                case USB_CS_RSTAT:
                    log_debug("control read complete\n");
                    usb.ep0_state = USB_CS_IDLE;
                    break;

                default:
                    log_error("OUT: unexpected event\n");
                    usb.ep0_state = USB_CS_IDLE;
                    usb_stall_ep0();
            }
            usb_arm_ep0out();
            break;

        default:
            log_error("unsupported pid=%d\n", pid);
            break;
    }
}

void usb_init_with_desc(const usb_desc_table_elem_t *desc_table) {
    memset(&usb_bdt, 0, sizeof(usb_bdt));
    memset(&usb, 0, sizeof(usb));
    U1CON = 0; /* first turn USB off */
    usb.desc_table = desc_table;
#ifndef USB_DISABLE_NONSTD_CTRL
    usb.ep0_nonstd_request_handler = usb_default_ep0_request_cb;
    usb.ep0_nonstd_data_complete = NULL;
#endif

    U1OTGCON = 0; /* turn off VUSB, disable special USB OTG functions */
    U1PWRC = _U1PWRC_USBPWR_MASK;

    /* disable all endpoints */
    int i;
    for (i = 0; i < 16; i++) {
        volatile unsigned *u1ep_reg = &U1EP0 + 4 * i; /* address of EP control reg */
        *u1ep_reg = 0;
    }

    U1BDTP3 = usb_virt_to_phys(&usb_bdt) >> 24;
    U1BDTP2 = usb_virt_to_phys(&usb_bdt) >> 16;
    U1BDTP1 = usb_virt_to_phys(&usb_bdt) >> 8;

#ifndef USB_DISABLE_IRQ
    /* enable interrupts */
    U1IE = _U1IE_TRNIE_MASK | _U1IE_STALLIE_MASK | _U1IE_URSTIE_MASK;
    usb_ei();

#if defined(__32MX440F256H__)
    IPC11bits.USBIP = 4;
#elif defined(__32MX470F512H__) || defined(__32MX250F128B__) || defined(__32MX270F256B__) || defined(__32MX270F256D__) || defined(__32MX274F256B__)
    IPC7bits.USBIP = 4;
#else
#error MCU type not supported
#endif
#endif

    /* endpoint zero */
    usb.ep0_new_address = 0;
    usb_init_endpoint(0x00, USB9_EPDESC_ATTR_TYPE_CONTROL, USB_EP0_SIZE, usb_ep0_handler);
    usb_init_endpoint(0x80, USB9_EPDESC_ATTR_TYPE_CONTROL, USB_EP0_SIZE, usb_ep0_handler);
    usb_arm_ep0out();

    U1CON = _U1CON_USBEN_MASK;
}

#ifndef USB_DISABLE_NONSTD_CTRL
void usb_set_nonstd_req_handler(usb_crtl_setup_cb_t setup_cb,
                                usb_ctrl_data_complete_cb_t data_complete_cb) {
    usb.ep0_nonstd_request_handler =
        setup_cb != NULL ? setup_cb : usb_default_ep0_request_cb;
    usb.ep0_nonstd_data_complete = data_complete_cb;
}
#endif

void usb_shutdown(void) {
    U1PWRC = 0;
#ifndef USB_DISABLE_IRQ
    U1IE = 0;
    usb_di();
#endif
}
