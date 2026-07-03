#include "esc/oneshot.h"

const struct esc_pwm_config oneshot42 = {
    .min_duty_us = 42, .max_duty_us = 84, .period_us = 1000};

const struct esc_pwm_config oneshot125 = {
    .min_duty_us = 125, .max_duty_us = 250, .period_us = 1000};
