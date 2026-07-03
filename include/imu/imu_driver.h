#ifndef FC_IMU_DRIVER_H
#define FC_IMU_DRIVER_H

/*
 * Generic IMU driver.
 */

#include <stdint.h>

/* 6-DoF IMU measurement in SI units. */
struct imu_si {
    /* Gyroscope: */
    float gx, gy, gz;
    /* Accelerometer: */
    float ax, ay, az;
};

struct imu_driver {
    void (*imu_setup)(void);

    /* Fetches new data if ready, else returns latest measurement. */
    struct imu_si *(*get_or_fetch_si)(void);

    /* Returns 0 if no new sample else timestamp when gyro data became ready. */
    uint32_t (*data_ready)(void);
};

#endif /* FC_IMU_DRIVER_H */
