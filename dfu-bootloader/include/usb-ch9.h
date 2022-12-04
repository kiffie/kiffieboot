/** @file
 * Macros and structures to implement USB Chapter 9 (and related stuff)
 *
 * Copyright (C) 2019 Kiffie
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#ifndef __USB_CH9_H__
#define	__USB_CH9_H__

/* bit masks for bmRequestType */
#define USB9_RT_IN                          0x80

#define USB9_RT_TYPE_MASK                   0x60
#define USB9_RT_TYPE_POS                    5
#define USB9_RT_TYPE_STANDARD               (0 << USB9_RT_TYPE_POS)
#define USB9_RT_TYPE_CLASS                  (1 << USB9_RT_TYPE_POS)
#define USB9_RT_TYPE_VENDOR                 (2 << USB9_RT_TYPE_POS)

#define USB9_RT_REC_MASK                    0x1f
#define USB9_RT_REC_POS                     0
#define USB9_RT_REC_DEVICE                  0
#define USB9_RT_REC_INTERFACE               1
#define USB9_RT_REC_ENDPOINT                2
#define USB9_RT_REC_OTHER                   3

#define USB9_REQ_GET_STATUS                 0
#define USB9_REQ_CLEAR_FEATURE              1
#define USB9_REQ_SET_FEATURE                3
#define USB9_REQ_SET_ADDRESS                5
#define USB9_REQ_GET_DESCRIPTOR             6
#define USB9_REQ_SET_DESCRIPTOR             7
#define USB9_REQ_GET_CONFIGURATION          8
#define USB9_REQ_SET_CONFIGURATION          9
#define USB9_REQ_GET_INTERFACE              10
#define USB9_REQ_SET_INTERFACE              11
#define USB9_REQ_SYNCH_FRAME                12

#define USB9_DESC_DEVICE                    1
#define USB9_DESC_CONFIGURATION             2
#define USB9_DESC_STRING                    3
#define USB9_DESC_INTERFACE                 4
#define USB9_DESC_ENDPOINT                  5
#define USB9_DESC_DEVICE_QUALIFIER          6
#define USB9_DESC_OTHER_SPEED_CONFIGURATION 8

/* HID descriptors */
#define USB9_DESC_HID                       0x21
#define USB9_DESC_HID_REPORT                0x22
#define USB9_DESC_HID_PHYS                  0x23

#define USB9_EPDESC_ATTR_TYPE_MASK          0x03
#define USB9_EPDESC_ATTR_TYPE_CONTROL       0
#define USB9_EPDESC_ATTR_TYPE_ISO           1
#define USB9_EPDESC_ATTR_TYPE_BULK          2
#define USB9_EPDESC_ATTR_TYPE_INTERRUPT     3

/* USB LANGIDs */
#define USB9_LANGID_EN_US                   0x0409

#define USB9_DESC_STRING_HEADER(len) ((2+2*(len)) | (USB9_DESC_STRING<<8))

/**
 * Data packet following an USB SETUP token.
 */
typedef struct __attribute__ ((__packed__)) usb9_setup_data {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb9_setup_data_t;

static inline bool usb9_is_std_request(uint8_t bmRequestType){
    return (bmRequestType & USB9_RT_TYPE_MASK) == USB9_RT_TYPE_STANDARD;
}

static inline bool usb9_is_vendor_request(uint8_t bmRequestType){
    return (bmRequestType & USB9_RT_TYPE_MASK) == USB9_RT_TYPE_VENDOR;
}

static inline bool usb9_is_class_request(uint8_t bmRequestType){
    return (bmRequestType & USB9_RT_TYPE_MASK) == USB9_RT_TYPE_CLASS;
}


typedef struct __attribute__ ((__packed__)) usb9_device_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
} usb9_device_descriptor_t;

struct __attribute__ ((__packed__)) usb9_configuration_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
};

struct __attribute__ ((__packed__)) usb9_interface_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
};

struct __attribute__ ((__packed__)) usb9_endpoint_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
};

/* HID descriptor for a single subordinate descriptor; additional pairs
 * of bDescriptorType2 and wDescriptorLength2 can be appended */
struct __attribute__ ((__packed__)) usb9_hid_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdHID;
    uint8_t bCountryCode;
    uint8_t bNumDescriptors;
    uint8_t bDescriptorType2;
    uint16_t wDescriptorLength2;
};

/*
 * needed for string descriptors
 */
struct __attribute__ ((__packed__)) usb9_descriptor_header {
    uint8_t bLength;
    uint8_t bDescriptorType;
};

#endif	/* USB_CH9_H */
