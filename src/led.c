#include "led.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <stdbool.h>

/* LED is connected to PC13. */
#define LED_RCC RCC_GPIOC
#define LED_PORT GPIOC
#define LED_PIN GPIO13

static bool initialized = false;

void led_setup(void)
{
    if (initialized) {
        return;
    }

    rcc_periph_clock_enable(LED_RCC);
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);

    initialized = true;
}

void led_toggle(void)
{
    gpio_toggle(LED_PORT, LED_PIN);
}
