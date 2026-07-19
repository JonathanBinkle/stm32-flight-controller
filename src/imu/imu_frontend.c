#include "imu/imu_frontend.h"
#include "constants.h"
#include "imu/imu_driver.h"
#include "timer.h"
#include <string.h>

#define CALIBRATION_SAMPLES 1e5

static struct imu_sample latest_sample;
static uint32_t prev_gyro_drdy_timestamp = 0;
static struct imu_bias bias;

struct imu_sample *imu_get_latest(const struct imu_driver *imu)
{
    uint32_t gyro_drdy_timestamp = imu->data_ready();

    if (!gyro_drdy_timestamp || !prev_gyro_drdy_timestamp) {
        /* No new sample, or no previous sample yet. */
        prev_gyro_drdy_timestamp = gyro_drdy_timestamp;
        return NULL;
    }

    latest_sample.dt = gyro_drdy_timestamp - prev_gyro_drdy_timestamp;
    prev_gyro_drdy_timestamp = gyro_drdy_timestamp;

    latest_sample.si = *imu->get_or_fetch_si();

    /* Remove static bias. */
    latest_sample.si.ax -= bias.ax;
    latest_sample.si.ay -= bias.ay;
    latest_sample.si.az -= bias.az;
    latest_sample.si.gx -= bias.gx;
    latest_sample.si.gy -= bias.gy;
    latest_sample.si.gz -= bias.gz;

    return &latest_sample;
}

void imu_calibrate(const struct imu_driver *imu, bool use_precomputed_bias)
{

    if (use_precomputed_bias) {
        bias.gx = 0.0012803464;
        bias.gy = -0.0045885462;
        bias.gz = -0.0086432267;
        bias.ax = 0.2003362477;
        bias.ay = 0.0798646286;
        bias.az = 0.1923642159;
    } else {
        struct imu_si sample;
        memset(&bias, 0, sizeof bias);

        for (uint32_t i = 0; i < CALIBRATION_SAMPLES; i++) {
            while (!imu->data_ready()) {
                timer_wait_us(500u); /* ODR 1.67kHz => sample every ~600us */
            }
            sample = *imu->get_or_fetch_si();

            bias.gx += sample.gx;
            bias.gy += sample.gy;
            bias.gz += sample.gz;
            bias.ax += sample.ax;
            bias.ay += sample.ay;
            bias.az += sample.az;
        }

        bias.gx /= CALIBRATION_SAMPLES;
        bias.gy /= CALIBRATION_SAMPLES;
        bias.gz /= CALIBRATION_SAMPLES;
        bias.ax /= CALIBRATION_SAMPLES;
        bias.ay /= CALIBRATION_SAMPLES;
        bias.az = (bias.az / CALIBRATION_SAMPLES) - GRAVITY_EARTH;
    }
}

struct imu_bias *imu_get_bias(void)
{
    return &bias;
}
