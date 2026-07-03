#ifndef FC_ATTITUDE_H
#define FC_ATTITUDE_H

/*
 * Attitude control.
 */

#include "imu/imu_frontend.h"

struct imu_angles {
    float fused_pitch, fused_roll;
    /* Accelerometer/Gyro angle estimation; kept for debugging: */
    float apitch, aroll;
    float gpitch, groll;
};

struct imu_angles *update_complementary_filter(struct imu_sample *sample);

#endif /* FC_ATTITUDE_H */
