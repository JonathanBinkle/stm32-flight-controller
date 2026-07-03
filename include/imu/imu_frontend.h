#ifndef FC_IMU_FRONTEND_H
#define FC_IMU_FRONTEND_H

/*
 * Bundles hardware-independent IMU behavior.
 */

#include "imu/imu_driver.h"
#include <stdbool.h>
#include <stdint.h>

struct imu_sample {
    struct imu_si si;
    uint32_t dt;
};

void imu_calibrate(const struct imu_driver *imu, bool use_precomputed_bias);
struct imu_sample *imu_get_latest(const struct imu_driver *imu);

/* Debug-only: */
struct imu_bias {
    float gx, gy, gz;
    float ax, ay, az;
};
struct imu_bias *imu_get_bias(void);

#endif /* FC_IMU_FRONTEND_H */
