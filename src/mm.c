#include "mm.h"
#include "esc/esc.h"
#include "rx/rx.h"
#include "util.h"

/* Disarm motors if RX throttle is near mininum. */
#define CUTOFF_THROTTLE ((uint16_t)(RX_RANGE_MIN + 0.01f * RX_RANGE))

/* Authority of PID corrections. */
#define PID_SCALER 2.5f

static struct esc_throttles out;

static void disarm(struct esc_throttles *throttles)
{
    throttles->front_left = ESC_MIN_THROTTLE;
    throttles->front_right = ESC_MIN_THROTTLE;
    throttles->back_left = ESC_MIN_THROTTLE;
    throttles->back_right = ESC_MIN_THROTTLE;
}

struct esc_throttles *motor_mixing(float roll, float pitch, float yaw,
                                   struct rx_sample *rx)
{
    if (rx->throttle <= CUTOFF_THROTTLE) {
        disarm(&out);
        return &out;
    }

    out.front_left = PID_SCALER * (-pitch + roll - yaw) + rx->throttle;
    out.front_right = PID_SCALER * (-pitch - roll + yaw) + rx->throttle;
    out.back_left = PID_SCALER * (+pitch + roll + yaw) + rx->throttle;
    out.back_right = PID_SCALER * (+pitch - roll - yaw) + rx->throttle;

    out.front_left = clamp(out.front_left, ESC_MIN_THROTTLE, ESC_MAX_THROTTLE);
    out.front_right =
        clamp(out.front_right, ESC_MIN_THROTTLE, ESC_MAX_THROTTLE);
    out.back_left = clamp(out.back_left, ESC_MIN_THROTTLE, ESC_MAX_THROTTLE);
    out.back_right = clamp(out.back_right, ESC_MIN_THROTTLE, ESC_MAX_THROTTLE);

    return &out;
}
