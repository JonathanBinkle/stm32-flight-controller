#include "esc/pwm.h"
#include "esc/esc.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <stdbool.h>

/* TIM5 generates PWM signals. PA0..3 with AF02 correspond to TIM5_CH1..4. */
#define PWM_TIM TIM5
#define PWM_TIM_NVIC NVIC_TIM5_IRQ
#define PWM_TIM_ISR tim5_isr
#define PWM_TIM_RCC RCC_TIM5
#define PWM_GPIO_PORT GPIOA
#define PWM_GPIO_CH1 GPIO0
#define PWM_GPIO_CH2 GPIO1
#define PWM_GPIO_CH3 GPIO2
#define PWM_GPIO_CH4 GPIO3
#define PWM_GPIO_AF GPIO_AF2
#define PWM_GPIO_RCC RCC_GPIOA

#define TIM_TICKS_PER_US (rcc_get_timer_clk_freq(PWM_TIM) / 1000000u)

/* Timer ticks. */
typedef uint32_t ticks_t;

static void setup(const void *ctx);
static void gpio_setup(void);
static void timer_setup(const struct esc_pwm_config *conf);

/* Set values for next ESC update. */
static void update(const void *ctx, const struct esc_throttles *throttles);

/* Maps THROTTLE from `[ESC_MIN_THROTTLE, ESC_MAX_THROTTLE]` to number of ticks
 * that PWM signal has to stay HIGH. */
static ticks_t throttle_to_ticks(esc_t t, const struct esc_pwm_config *conf);

const struct esc_driver pwm_driver = {.esc_setup = setup, .esc_update = update};

/* Throttles (in #ticks that PWM is HIGH) to send on next update to ESCs. */
static volatile struct {
    ticks_t front_left;
    ticks_t front_right;
    ticks_t back_left;
    ticks_t back_right;
} pulses;

static inline const struct esc_pwm_config *pwm_conf(const void *ctx)
{
    const struct esc_pwm_driver *self = ctx;
    return &self->pwm_config;
}

static void gpio_setup(void)
{
    const uint16_t channels =
        (PWM_GPIO_CH1 | PWM_GPIO_CH2 | PWM_GPIO_CH3 | PWM_GPIO_CH4);
    rcc_periph_clock_enable(PWM_GPIO_RCC);
    gpio_mode_setup(PWM_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, channels);
    gpio_set_af(PWM_GPIO_PORT, PWM_GPIO_AF, channels);
}

static inline uint32_t us_to_ticks(float us)
{
    return (uint32_t)(TIM_TICKS_PER_US * us);
}

static void timer_setup(const struct esc_pwm_config *cfg)
{
    rcc_periph_clock_enable(PWM_TIM_RCC);

    /* Use internal clock; count up on edges. */
    timer_set_mode(PWM_TIM, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);

    /* Set ARR to number of ticks per OneShot period. */
    timer_set_prescaler(PWM_TIM, 0); /* No prescaler. */
    timer_set_period(PWM_TIM, TIM_TICKS_PER_US * cfg->period_us - 1);
    timer_set_counter(PWM_TIM, 0);
    TIM_EGR(PWM_TIM) = TIM_EGR_UG; /* UEV to immediately apply settings. */

    /* Output PWM signal on each channel. LOW if timer's counter < CCR. */
    timer_set_oc_mode(PWM_TIM, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_mode(PWM_TIM, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_mode(PWM_TIM, TIM_OC3, TIM_OCM_PWM1);
    timer_set_oc_mode(PWM_TIM, TIM_OC4, TIM_OCM_PWM1);

    /* Initialize ESCs to no throttle. */
    const ticks_t min_throttle = throttle_to_ticks(ESC_MIN_THROTTLE, cfg);
    timer_set_oc_value(PWM_TIM, TIM_OC1, min_throttle);
    timer_set_oc_value(PWM_TIM, TIM_OC2, min_throttle);
    timer_set_oc_value(PWM_TIM, TIM_OC3, min_throttle);
    timer_set_oc_value(PWM_TIM, TIM_OC4, min_throttle);

    /* Enable output. */
    timer_enable_oc_output(PWM_TIM, TIM_OC1);
    timer_enable_oc_output(PWM_TIM, TIM_OC2);
    timer_enable_oc_output(PWM_TIM, TIM_OC3);
    timer_enable_oc_output(PWM_TIM, TIM_OC4);

    /* Enable interrupts on UEV. */
    timer_clear_flag(PWM_TIM, TIM_SR_UIF);
    timer_enable_irq(PWM_TIM, TIM_DIER_UIE);
    nvic_enable_irq(PWM_TIM_NVIC);

    /* Start timer. */
    timer_enable_counter(PWM_TIM);
}

static inline void esc_set_pulse(enum esc_position pos, uint16_t ticks)
{
    timer_set_oc_value(PWM_TIM, esc_map[pos], ticks);
}

void PWM_TIM_ISR(void)
{
    if (timer_get_flag(PWM_TIM, TIM_SR_UIF)) {
        timer_clear_flag(PWM_TIM, TIM_SR_UIF);
        esc_set_pulse(FRONT_LEFT, pulses.front_left);
        esc_set_pulse(FRONT_RIGHT, pulses.front_right);
        esc_set_pulse(BACK_LEFT, pulses.back_left);
        esc_set_pulse(BACK_RIGHT, pulses.back_right);
    }
}

static void setup(const void *ctx)
{
    const struct esc_pwm_config *cfg = pwm_conf(ctx);

    const ticks_t min_throttle = throttle_to_ticks(ESC_MIN_THROTTLE, cfg);
    pulses.front_left = min_throttle;
    pulses.front_right = min_throttle;
    pulses.back_left = min_throttle;
    pulses.back_right = min_throttle;

    gpio_setup();
    timer_setup(cfg);
}

static ticks_t throttle_to_ticks(esc_t t, const struct esc_pwm_config *conf)
{
    float perc = (float)(t - ESC_MIN_THROTTLE) /
                 (float)(ESC_MAX_THROTTLE - ESC_MIN_THROTTLE);
    return us_to_ticks((perc * (conf->max_duty_us - conf->min_duty_us)) +
                       conf->min_duty_us);
}

static inline bool valid_throttle_range(esc_t throttle)
{
    return ESC_MIN_THROTTLE <= throttle && throttle <= ESC_MAX_THROTTLE;
}

static void update(const void *ctx, const struct esc_throttles *t)
{
    const struct esc_pwm_config *cfg = pwm_conf(ctx);

    /* NOTE: we could get interrupted in the following, such that a mixed
     * old/new `pulse_ticks` is written to ESCs. That's okay, don't handle. */
    if (valid_throttle_range(t->front_left)) {
        pulses.front_left = throttle_to_ticks(t->front_left, cfg);
    }
    if (valid_throttle_range(t->front_right)) {
        pulses.front_right = throttle_to_ticks(t->front_right, cfg);
    }
    if (valid_throttle_range(t->back_left)) {
        pulses.back_left = throttle_to_ticks(t->back_left, cfg);
    }
    if (valid_throttle_range(t->back_right)) {
        pulses.back_right = throttle_to_ticks(t->back_right, cfg);
    }
}
