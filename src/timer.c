#include "timer.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <stdbool.h>

/* Use TIM2 peripheral (32-bit timer). Wraps every ~71 minutes. */
#define TIMER TIM2
#define TIMER_RCC RCC_TIM2
#define TIMER_IRQ NVIC_TIM2_IRQ

/* Busy-wait if remaining time to sleep is below this. */
#define MIN_SLEEP_US 30u

static bool initialized = false;

void timer_setup(void)
{
    if (initialized) {
        return;
    }

    rcc_periph_clock_enable(TIMER_RCC);

    /* 1 MHz timer -> 1 tick per us. Counts 0..2^32-1 then wraps. */
    timer_set_mode(TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIMER, (rcc_get_timer_clk_freq(TIMER) / 1000000u) - 1);
    timer_set_period(TIMER, 0xffffffff);
    timer_set_counter(TIMER, 0);
    TIM_EGR(TIMER) = TIM_EGR_UG; /* Force UEV (apply settings immediately). */

    /* Fire IRQ when we hit OC1; OC1 will be set to deadline. */
    timer_disable_oc_output(TIMER, TIM_OC1);
    timer_set_oc_mode(TIMER, TIM_OC1, TIM_OCM_FROZEN);
    timer_disable_irq(TIMER, TIM_DIER_CC1IE);
    nvic_enable_irq(TIMER_IRQ);

    timer_enable_counter(TIMER);

    initialized = true;
}

void timer_wait_us(uint32_t us)
{
    timer_wait_until_us(timer_get_counter(TIMER) + us);
}

uint32_t timer_now_us(void)
{
    return timer_get_counter(TIMER);
}

void timer_wait_until_us(uint32_t deadline_us)
{
    while (1) {
        uint32_t now = timer_get_counter(TIMER);
        int32_t remaining = (int32_t)(deadline_us - now);

        if (remaining <= 0) {
            return;
        }

        if ((uint32_t)remaining <= MIN_SLEEP_US) {
            /* Close enough that sleeping isn't worth it - busy-wait. */
            while ((int32_t)(deadline_us - timer_get_counter(TIMER)) > 0) {
                ;
            }
            return;
        }

        timer_set_oc_value(TIMER, TIM_OC1, deadline_us);
        timer_clear_flag(TIMER, TIM_SR_CC1IF); /* clear stale */
        timer_enable_irq(TIMER, TIM_DIER_CC1IE);
        __asm__ volatile("wfi");
        timer_disable_irq(TIMER, TIM_DIER_CC1IE);

        /* Loop back around and handle `remaining` again: we may have waken up
         * for an unrelated IRQ and have to sleep more. */
    }
}

void tim2_isr(void)
{
    if (TIM_SR(TIMER) & TIM_SR_CC1IF) {
        TIM_SR(TIMER) = ~TIM_SR_CC1IF;
    }
}
