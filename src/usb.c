/* Adjusted from:
 * https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f4/stm32f4-discovery/usb_cdcacm/cdcacm.c
 */

#include "usb.h"
#include "led.h"
#include "timer.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/usb/dwc/otg_common.h>
#include <libopencm3/usb/dwc/otg_fs.h>
#include <libopencm3/usb/usbd.h>
#include <stdbool.h>

#define USB_RCC RCC_OTGFS
#define USB_RX_ENDPOINT_ADDR 0x01
#define USB_TX_ENDPOINT_ADDR 0x82
#define USB_COMM_ENDPOINT_ADDR 0x83

/* GPIO: PA11 (USB_FS_DM) and PA12 (USB_FS_DP). */
#define USB_GPIO_PORT GPIOA
#define USB_GPIO_DM GPIO11
#define USB_GPIO_DP GPIO12
#define USB_GPIO_AF GPIO_AF10
#define USB_GPIO_RCC RCC_GPIOA

static volatile bool tx_ready = true; /* TX endpoint ready for next packet? */

static bool initialized = false;
static usbd_device *_usbd_dev;
static const char *usb_strings[] = {
    "Jonathan",   /* iManufacturer */
    "Quadcopter", /* iProduct */
    "v1",         /* iSerial */
};
static uint8_t usbd_control_buffer[128];

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = USB_MAX_PACKET_SIZE,
    .idVendor = 0x0483,  /* STMicroelectronics */
    .idProduct = 0x5740, /* Virtual COM Port */
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

/* This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver. */
static const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_COMM_ENDPOINT_ADDR,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {
    /* Rx endpoint */
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = USB_RX_ENDPOINT_ADDR,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = USB_MAX_PACKET_SIZE,
        .bInterval = 1,
    },

    /* Tx endpoint */
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = USB_TX_ENDPOINT_ADDR,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = USB_MAX_PACKET_SIZE,
        .bInterval = 1,
    }};

static const struct {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
    .header =
        {
            .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
            .bcdCDC = 0x0110,
        },
    .call_mgmt =
        {
            .bFunctionLength =
                sizeof(struct usb_cdc_call_management_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
            .bmCapabilities = 0,
            .bDataInterface = 1,
        },
    .acm =
        {
            .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_ACM,
            .bmCapabilities = 0,
        },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
    }};

static const struct usb_interface_descriptor comm_iface[] = {
    {.bLength = USB_DT_INTERFACE_SIZE,
     .bDescriptorType = USB_DT_INTERFACE,
     .bInterfaceNumber = 0,
     .bAlternateSetting = 0,
     .bNumEndpoints = 1,
     .bInterfaceClass = USB_CLASS_CDC,
     .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
     .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
     .iInterface = 0,
     .endpoint = comm_endp,
     .extra = &cdcacm_functional_descriptors,
     .extralen = sizeof(cdcacm_functional_descriptors)}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
    .endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
                                                  .num_altsetting = 1,
                                                  .altsetting = comm_iface,
                                              },
                                              {
                                                  .num_altsetting = 1,
                                                  .altsetting = data_iface,
                                              }};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,

    .interface = ifaces,
};

static void tx_callback(usbd_device *usbd_dev, uint8_t ep)
{
    /* Called when TX endpoint is done with previous transaction. */
    (void)usbd_dev;
    (void)ep;
    tx_ready = true;
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    /* Rx endpoint */
    usbd_ep_setup(usbd_dev, USB_RX_ENDPOINT_ADDR, USB_ENDPOINT_ATTR_BULK,
                  USB_MAX_PACKET_SIZE, NULL);

    /* Tx endpoint */
    usbd_ep_setup(usbd_dev, USB_TX_ENDPOINT_ADDR, USB_ENDPOINT_ATTR_BULK,
                  USB_MAX_PACKET_SIZE, tx_callback);

    /* Control endpoint */
    usbd_ep_setup(usbd_dev, USB_COMM_ENDPOINT_ADDR, USB_ENDPOINT_ATTR_INTERRUPT,
                  16, NULL);
}

void otg_fs_isr(void)
{
    usbd_poll(_usbd_dev);
}

void usb_setup(void)
{
    if (initialized) {
        return;
    }

    rcc_periph_clock_enable(USB_GPIO_RCC);
    rcc_periph_clock_enable(USB_RCC);
    gpio_mode_setup(USB_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    (USB_GPIO_DP | USB_GPIO_DM));
    gpio_set_af(USB_GPIO_PORT, USB_GPIO_AF, (USB_GPIO_DP | USB_GPIO_DM));

    /* Disable VBUS sensing check as the BlackPill doesn't connect the USB
     * Type-C connector's VBUS pin to VBUS sensing pin (PA9). See:
     * - RM0383 page 718
     * - https://community.platformio.org/t/36608 */
    OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS;

    _usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, 3,
                          usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(_usbd_dev, cdcacm_set_config);

    nvic_enable_irq(NVIC_OTG_FS_IRQ);

    initialized = true;
}

void usb_send(const void *buf, size_t n, uint32_t deadline_us)
{
    const uint8_t *p = buf;

    while (n > 0 && timer_now_us() < deadline_us) {
        size_t chunk = (n > USB_MAX_PACKET_SIZE) ? USB_MAX_PACKET_SIZE : n;

        while (!tx_ready && timer_now_us() < deadline_us) {
            /* wait for previous transaction to complete */
        }

        if (!tx_ready) {
            return; /* hit deadline while waiting for tx_ready */
        }

        nvic_disable_irq(NVIC_OTG_FS_IRQ);
        tx_ready = false;
        usbd_ep_write_packet(_usbd_dev, USB_TX_ENDPOINT_ADDR, p, chunk);
        nvic_enable_irq(NVIC_OTG_FS_IRQ);

        p += chunk;
        n -= chunk;
    }
}
