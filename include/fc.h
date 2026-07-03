#ifndef FC_H
#define FC_H

/*
 * Flight controller.
 */

#include "attitude.h"
#include "imu/imu_driver.h"
#include "pid.h"
#include "rx/rx.h"
#include <stdint.h>

enum flight_mode {
    MODE_STABILIZE,
    MODE_ACRO
};

/* Flight controller state. */
struct fc {
    struct pid pitch_pid;
    struct pid roll_pid;
    struct pid yaw_pid;
    uint8_t mode; /* enum flight_mode but fixed type size */
};

/* Output of flight controller, i.e., control signals. */
struct fc_out {
    float pitch_out;
    float roll_out;
    float yaw_out;
};

void fc_setup(enum flight_mode mode);
struct fc_out *fc_update(const struct rx_sample *rx,
                         const struct imu_angles *imu_angles,
                         const struct imu_si *imu_si, uint32_t now_us);

/* Debug only. */
enum pid_axis {
    PITCH,
    ROLL,
    YAW
};
struct pid *fc_get_pid(enum pid_axis pid);

#endif /* FC_H */
