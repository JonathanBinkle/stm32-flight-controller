#include "led.h"
#include "timer.h"
#include <libopencm3/stm32/rcc.h>
#include <stdbool.h>

int main(void)
{
    /* Clock setup.
     * Use 25 MHz HSE crystal as PLL input.
     * Configure the PLL to output a 84 MHz SYSCLK and 48 MHz USB clock. */
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);

    timer_setup();
    led_setup();

    while (true) {
        led_toggle();
        timer_wait_us(1000000);
    }
}
