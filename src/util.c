#include "util.h"
#include "led.h"
#include "timer.h"
#include "usb.h"
#include <string.h>

void error(const char *msg)
{
    uint32_t i = 0;
    while (1) {
        led_toggle();
        timer_wait_us(2e5);
        if (i++ % 10) { /* Don't overwhelm USB. */
            usb_send(msg, strlen(msg), /* 1s timeout: */ timer_now_us() + 1e6);
        }
    }
}

float map_into_range(float src_min, float src_max, float val, float dst_min,
                     float dst_max)
{
    float perc = (val - src_min) / (src_max - src_min);
    return dst_min + perc * (dst_max - dst_min);
}

float clamp(float val, float min, float max)
{
    if (val < min) {
        val = min;
    } else if (val > max) {
        val = max;
    }

    return val;
}
