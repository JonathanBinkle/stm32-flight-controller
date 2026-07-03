#ifndef FC_USB_H
#define FC_USB_H

/*
 * USB CDC-ACM device.
 */

#include <stddef.h>
#include <stdint.h>

#define USB_MAX_PACKET_SIZE 64u

void usb_setup(void);

/* Send `n` bytes from `buf` over USB. Abort if time now >= `deadline_us`. */
void usb_send(const void *buf, size_t n, uint32_t deadline_us);

#endif /* FC_USB_H */
