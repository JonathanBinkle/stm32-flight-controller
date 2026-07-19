#ifndef FC_ESC_H
#define FC_ESC_H

/*
 * Generic ESC driver.
 */

#include <libopencm3/stm32/timer.h>
#include <stdint.h>

enum esc_position {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
};

/* Map ESC position to timer's output channel. Depends on hardware wiring. */
static const enum tim_oc_id esc_map[4] = {
    [FRONT_LEFT] = TIM_OC3,
    [FRONT_RIGHT] = TIM_OC2,
    [BACK_LEFT] = TIM_OC4,
    [BACK_RIGHT] = TIM_OC1,
};

typedef uint16_t esc_t;
enum {
    ESC_MIN_THROTTLE = 1000u,
    ESC_MAX_THROTTLE = 2000u,
    ESC_THROTTLE_RANGE = ESC_MAX_THROTTLE - ESC_MIN_THROTTLE
};

struct esc_throttles {
    esc_t front_left;
    esc_t front_right;
    esc_t back_left;
    esc_t back_right;
};

struct esc_driver {
    void (*esc_setup)(const void *ctx);
    void (*esc_update)(const void *ctx, const struct esc_throttles *throttles);
};

#endif /* FC_ESC_H */
