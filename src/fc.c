#include "fc.h"
#include "attitude.h"
#include "constants.h"
#include "esc/esc.h"
#include "pid.h"
#include "telemetry/telemetry.h"
#include "util.h"
#include <stdint.h>

static struct fc fc;
static struct fc_out fc_out;

/* Roughly the throttle it takes to lift off. */
#define TAKEOFF_THROTTLE                                                       \
    (uint16_t)(0.2f * (ESC_MAX_THROTTLE - ESC_MIN_THROTTLE) + ESC_MIN_THROTTLE)

/* Clamp I-term contribution to 10% of current throttle as anti-windup. */
#define ITERM_LIMIT(throttle) (0.1f * (throttle))

/* Flight mode parameters. */
#define MIN_ROLL_PITCH_ANGLE_DEGREES -25.0f
#define MAX_ROLL_PITCH_ANGLE_DEGREES 25.0f
#define MIN_RATE_DPS -125.0f
#define MAX_RATE_DPS 125.0f

/* Tuned PID gains. */
#define KP_ROLL_PITCH 2.0f
#define KI_ROLL_PITCH 0.5f
#define KD_ROLL_PITCH 0.25f
#define KP_YAW 1.5f
#define KI_YAW 0.0f
#define KD_YAW 0.0f

static inline float rx_to_angles(float rx)
{
    return map_into_range(RX_RANGE_MIN, RX_RANGE_MAX, rx,
                          MIN_ROLL_PITCH_ANGLE_DEGREES,
                          MAX_ROLL_PITCH_ANGLE_DEGREES);
}

static inline float rx_to_rate(float rx)
{
    return map_into_range(RX_RANGE_MIN, RX_RANGE_MAX, rx, MIN_RATE_DPS,
                          MAX_RATE_DPS);
}

void fc_setup(enum flight_mode mode)
{
    pid_setup(&fc.pitch_pid, KP_ROLL_PITCH, KI_ROLL_PITCH, KD_ROLL_PITCH);
    pid_setup(&fc.roll_pid, KP_ROLL_PITCH, KI_ROLL_PITCH, KD_ROLL_PITCH);
    pid_setup(&fc.yaw_pid, KP_YAW, KI_YAW, KD_YAW);
    fc.mode = (uint8_t)mode;
}

struct fc_out *fc_update(const struct rx_sample *rx,
                         const struct imu_angles *imu_angles,
                         const struct imu_si *imu_si, uint32_t now_us)
{
    float pitch_desired, roll_desired, yaw_desired;
    float pitch_actual, roll_actual, yaw_actual;

    switch (fc.mode) {
        case MODE_STABILIZE:
            pitch_desired = rx_to_angles(rx->pitch);
            roll_desired = rx_to_angles(rx->roll);
            yaw_desired = rx_to_rate(rx->yaw);
            pitch_actual = imu_angles->fused_pitch;
            roll_actual = imu_angles->fused_roll;
            yaw_actual = imu_si->gz * RAD2DEG;
            break;
        case MODE_ACRO:
            pitch_desired = rx_to_rate(rx->pitch);
            roll_desired = rx_to_rate(rx->roll);
            yaw_desired = rx_to_rate(rx->yaw);
            pitch_actual = imu_si->gy * RAD2DEG;
            roll_actual = imu_si->gx * RAD2DEG;
            yaw_actual = imu_si->gz * RAD2DEG;
            break;
        default:
            error("[FC] unknown flight mode");
            break;
    }

    /* Only add to integral error if drone is airborne. Otherwise, me moving
     * drone by hand or drone sitting on ground unable to correct its error,
     * increases the integral error artificially. */
    bool integrate = rx->throttle > TAKEOFF_THROTTLE;

    fc_out.pitch_out = pid_update(pitch_desired, pitch_actual, &fc.pitch_pid,
                                  now_us, integrate, ITERM_LIMIT(rx->throttle));
    fc_out.roll_out = pid_update(roll_desired, roll_actual, &fc.roll_pid,
                                 now_us, integrate, ITERM_LIMIT(rx->throttle));
    fc_out.yaw_out = pid_update(yaw_desired, yaw_actual, &fc.yaw_pid, now_us,
                                integrate, ITERM_LIMIT(rx->throttle));

    return &fc_out;
}

struct pid *fc_get_pid(enum pid_axis pid)
{
    switch (pid) {
        case PITCH:
            return &fc.pitch_pid;
        case ROLL:
            return &fc.roll_pid;
        case YAW:
            return &fc.yaw_pid;
        default:
            error("[fc] unknown enum pid_axis member in fc_get_pid");
            return NULL;
    }
}
