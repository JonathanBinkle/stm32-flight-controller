#ifndef FC_TELEMETRY_H
#define FC_TELEMETRY_H

/*
 * Telemetry module.
 */

#include <stddef.h>
#include <stdint.h>

enum telemetry_id {
    RX_SAMPLE,     /* struct rx_sample */
    IMU_SI,        /* struct imu_si */
    IMU_SAMPLE,    /* struct imu_sample */
    IMU_ANGLES,    /* struct imu_angles */
    ESC_THROTTLES, /* struct esc_throttles */
    FC_OUT,        /* struct fc_out */
    PID_PITCH,     /* struct pid */
    PID_ROLL,      /* struct pid */
    PID_YAW,       /* struct pid */
    IMU_BIAS       /* struct imu_bias */
};

struct telemetry_header {
    uint32_t magic;  /* Packet delimiter */
    uint8_t id;      /* What is send (enum telemetry_id but fixed type width) */
    uint16_t len;    /* Payload length */
    uint32_t seqnum; /* Sequence number (inc'd each main loop iteration) */
} __attribute__((packed));

struct telemetry_backend {
    void (*setup)(void);
    void (*send)(enum telemetry_id id, const void *payload, uint16_t len,
                 uint32_t deadline_us);
};

void telemetry_tick(void);
uint32_t telemetry_get_seqnum(void);

#endif /* FC_TELEMETRY_H */
