#ifndef FC_MM_H
#define FC_MM_H

/*
 * Motor mixing.
 */

#include "rx/rx.h"

/* Given roll, pitch, yaw PID outputs and current throttle, output next motor
 * throttles. */
struct esc_throttles *motor_mixing(float roll, float pitch, float yaw,
                                   struct rx_sample *rx);

#endif /* FC_MM_H */
