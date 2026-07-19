#ifndef FC_RX_H
#define FC_RX_H

/*
 * Generic radio receiver (RX) driver.
 */

#include <stdbool.h>
#include <stdint.h>

typedef uint16_t rx_t;

/* Per-channel values range from 1000 to 2000 (standard PWM compliant). */
enum {
    RX_RANGE_MIN = 1000,
    RX_RANGE_MAX = 2000,
    RX_RANGE = (RX_RANGE_MAX - RX_RANGE_MIN)
};

/* RX data, supporting up to 14 channels. */
struct rx_sample {
    rx_t roll;
    rx_t pitch;
    rx_t throttle;
    rx_t yaw;
    rx_t ch5;
    rx_t ch6;
    rx_t ch7;
    rx_t ch8;
    rx_t ch9;
    rx_t ch10;
    rx_t ch11;
    rx_t ch12;
    rx_t ch13;
    rx_t ch14;
};

struct rx_driver {
    void (*rx_setup)(void);

    /* Latest RX data. Blocks until next sample if no sample has been received
     * yet or if `blocking` is set. */
    struct rx_sample *(*rx_recv)(bool blocking);

    /* Absolute time in microseconds of last received RX packet.
     * Returns 0 if no packet has been received yet. */
    uint32_t (*rx_time_last_rcvd_packet)(void);
};

#endif /* FC_RX_H */
