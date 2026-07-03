#include "telemetry/usb_telemetry.h"
#include "telemetry/telemetry.h"
#include "timer.h"
#include "usb.h"
#include <string.h>

#define MAGIC 0xdeadbeefu

static void send(enum telemetry_id id, const void *payload, uint16_t len,
                 uint32_t deadline_us)
{
    if (timer_now_us() >= deadline_us) {
        return;
    }

    struct telemetry_header header = {.magic = MAGIC,
                                      .id = (uint8_t)id,
                                      .len = len,
                                      .seqnum = telemetry_get_seqnum()};

    if (sizeof(header) + len <= USB_MAX_PACKET_SIZE) {
        /* Merge header & payload to minimize #calls to usb_send(). */
        uint8_t buf[USB_MAX_PACKET_SIZE];
        memcpy(buf, &header, sizeof header);
        memcpy(buf + sizeof header, payload, len);
        usb_send(buf, (sizeof header) + len, deadline_us);
    } else {
        usb_send(&header, sizeof header, deadline_us);
        usb_send(payload, len, deadline_us);
    }
}

const struct telemetry_backend usb_telemetry = {.setup = usb_setup,
                                                .send = send};
