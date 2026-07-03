#ifndef FC_ESC_PWM_H
#define FC_ESC_PWM_H

/*
 * Pulse Width Modulation (PWM) ESC driver.
 */

#include "esc/esc.h"

struct esc_pwm_config {
    uint32_t min_duty_us;
    uint32_t max_duty_us;
    uint32_t period_us;
};

struct esc_pwm_driver {
    const struct esc_driver *base;
    struct esc_pwm_config pwm_config;
};

extern const struct esc_driver pwm_driver;

#endif /* FC_ESC_PWM_H */
