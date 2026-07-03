#include "attitude.h"
#include "constants.h"
#include <math.h>

#define sq(n) (powf((n), 2.0f))

static struct imu_angles angles;

/* Accelerometer-only estimation of roll angle. Expects values in SI-unit. */
static inline float aroll(float ax, float ay, float az)
{
    return -atan2f(ay, sqrtf(sq(ax) + sq(az))) * RAD2DEG;
}

/* Accelerometer-only estimation of pitch angle. Expects values in SI-unit. */
static inline float apitch(float ax, float ay, float az)
{
    return -atan2f(ax, sqrtf(sq(ay) + sq(az))) * RAD2DEG;
}

/* Gyroscope-only estimation of roll angle. Expects `gx` in SI-unit, `dt_us` in
 * microseconds. */
static inline float groll(float gx, float dt_us)
{
    return gx * dt_us / 1e6f * RAD2DEG;
}

/* Gyroscope-only estimation of pitch angle. Expects `gy` in SI-unit, `dt_us` in
 * microseconds. */
static inline float gpitch(float gy, float dt_us)
{
    return gy * dt_us / 1e6f * RAD2DEG;
}

struct imu_angles *update_complementary_filter(struct imu_sample *sample)
{
    angles.aroll = aroll(sample->si.ax, sample->si.ay, sample->si.az);
    angles.apitch = apitch(sample->si.ax, sample->si.ay, sample->si.az);

    float _groll = groll(sample->si.gx, sample->dt);
    float _gpitch = gpitch(sample->si.gy, sample->dt);

    angles.groll += _groll;
    angles.gpitch += _gpitch;

    const float gweight = 0.995f;
    angles.fused_roll = (angles.fused_roll + _groll) * gweight +
                        angles.aroll * (1.0f - gweight);
    angles.fused_pitch = (angles.fused_pitch + _gpitch) * gweight +
                         angles.apitch * (1.0f - gweight);

    return &angles;
}
