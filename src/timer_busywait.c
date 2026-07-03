// #include "timer.h"
// #include <libopencm3/stm32/rcc.h>
// #include <libopencm3/stm32/timer.h>
// #include <stdbool.h>
//
///* Use TIM2 peripheral (32-bit timer). Wraps every ~71 minutes. */
// #define TIMER TIM2
// #define TIMER_RCC RCC_TIM2
//
// static bool initialized = false;
//
// void timer_setup(void)
//{
//     if (initialized) {
//         return;
//     }
//
//     rcc_periph_clock_enable(TIMER_RCC);
//
//     /* 1 MHz timer -> 1 tick per us. Counts 0..2^32-1 then wraps. */
//     timer_set_mode(TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
//     TIM_CR1_DIR_UP); timer_set_prescaler(TIMER,
//     (rcc_get_timer_clk_freq(TIMER) / 1e6) - 1); timer_set_period(TIMER,
//     0xffffffff); timer_set_counter(TIMER, 0); TIM_EGR(TIMER) = TIM_EGR_UG; /*
//     Force UEV (apply settings immediately). */ timer_enable_counter(TIMER);
//
//     initialized = true;
// }
//
// void timer_wait_us(uint32_t us)
//{
//     uint32_t start = timer_get_counter(TIMER);
//     while (timer_get_counter(TIMER) - start < us) {
//         ;
//     }
// }
//
// uint32_t timer_now_us(void)
//{
//     return timer_get_counter(TIMER);
// }
//
// void timer_wait_until_us(uint32_t us)
//{
//     while (timer_get_counter(TIMER) < us) {
//         ;
//     }
// }
