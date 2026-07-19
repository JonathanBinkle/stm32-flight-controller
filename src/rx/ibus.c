#include "rx/ibus.h"
#include "rx/rx.h"
#include "timer.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <stdbool.h>
#include <stdint.h>

/* Map axes to channel number. */
#define RX_CH_ROLL 1
#define RX_CH_PITCH 2
#define RX_CH_THROTTLE 3
#define RX_CH_YAW 4

/* iBus on USART1 (RX on PA10). */
#define IBUS_USART_GPIO_PORT GPIOA
#define IBUS_USART_GPIO_PIN_RX GPIO10
#define IBUS_USART_GPIO_RCC RCC_GPIOA
#define IBUS_USART_AF GPIO_AF7
#define IBUS_USART_RCC RCC_USART1
#define IBUS_USART_NVIC NVIC_USART1_IRQ
#define IBUS_USART USART1
#define IBUS_USART_ISR usart1_isr

static void ibus_setup(void);
static struct rx_sample *ibus_recv(bool blocking);
static uint32_t ibus_time_last_rcvd_packet(void);
const struct rx_driver ibus_driver = {.rx_setup = ibus_setup,
                                      .rx_recv = ibus_recv,
                                      .rx_time_last_rcvd_packet =
                                          ibus_time_last_rcvd_packet};

/* Processing state of the latest IBUS packet. */
static struct {
    uint8_t nbytes;
    uint16_t computed_cksum;
    uint16_t packet_cksum;
    uint8_t bytes[14 * 2]; /* 14 channels of 2 bytes each */
} ibus_state;

static volatile bool ibus_data_ready = false;
static volatile bool ibus_rcvd_first_packet = false;
static volatile uint32_t time_last_rcvd_packet = 0;
static struct rx_sample ibus_data;
static bool initialized = false;

static inline rx_t ibus_read_channel(uint8_t channel)
{
    return (ibus_state.bytes[(channel << 1) - 2] |
            (ibus_state.bytes[(channel << 1) - 1] << 8));
}

static void ibus_update_state(uint8_t ch)
{
    switch (ibus_state.nbytes) {
        case 0:
            if (ch == 0x20) {
                ibus_state.computed_cksum = 0xFFFF - ch;
                ibus_state.nbytes++;
            }
            break;
        case 1:
            if (ch == 0x40) {
                ibus_state.computed_cksum -= ch;
                ibus_state.nbytes++;
            } else {
                /* Invalid packet start. Reset state. */
                ibus_state.nbytes = 0;
            }
            break;
        case 30:
            ibus_state.packet_cksum = ch;
            ibus_state.nbytes++;
            break;
        case 31:
            if (ibus_state.computed_cksum ==
                ((ch << 8) | ibus_state.packet_cksum)) {

                /* Copy into rx_data. Do this here (within ISR) such that the
                 * main loop never sees mixed old/new rx_data (from a main loop
                 * perspective, this is an "atomic" copy). This is easier than
                 * temporarily disabling interrupts. */
                ibus_data.roll = ibus_read_channel(RX_CH_ROLL);
                ibus_data.pitch = ibus_read_channel(RX_CH_PITCH);
                ibus_data.throttle = ibus_read_channel(RX_CH_THROTTLE);
                ibus_data.yaw = ibus_read_channel(RX_CH_YAW);
                ibus_data.ch5 = ibus_read_channel(5);
                ibus_data.ch6 = ibus_read_channel(6);
                ibus_data.ch7 = ibus_read_channel(7);
                ibus_data.ch8 = ibus_read_channel(8);
                ibus_data.ch9 = ibus_read_channel(9);
                ibus_data.ch10 = ibus_read_channel(10);
                ibus_data.ch11 = ibus_read_channel(11);
                ibus_data.ch12 = ibus_read_channel(12);
                ibus_data.ch13 = ibus_read_channel(13);
                ibus_data.ch14 = ibus_read_channel(14);

                ibus_data_ready = true;
                ibus_rcvd_first_packet = true;
                time_last_rcvd_packet = timer_now_us();
            }

            /* Reset state for next packet. */
            ibus_state.nbytes = 0;
            break;
        default:
            ibus_state.computed_cksum -= ch;
            ibus_state.bytes[ibus_state.nbytes++ - 2] = ch;
    }
}

void IBUS_USART_ISR(void)
{
    uint32_t sr = USART_SR(IBUS_USART);

    /* Clear any USART error s.t. we don't get stuck. */
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        volatile uint8_t dummy = USART_DR(IBUS_USART);
        (void)dummy;
    }

    if (sr & USART_SR_RXNE) {
        ibus_update_state((uint8_t)usart_recv(IBUS_USART));
    }
}

static void ibus_setup(void)
{
    if (initialized) {
        return;
    }

    /* Setup GPIO for USART. */
    rcc_periph_clock_enable(IBUS_USART_GPIO_RCC);
    rcc_periph_clock_enable(IBUS_USART_RCC);
    gpio_mode_setup(IBUS_USART_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    IBUS_USART_GPIO_PIN_RX);
    gpio_set_output_options(IBUS_USART_GPIO_PORT, GPIO_OTYPE_OD,
                            GPIO_OSPEED_25MHZ, IBUS_USART_GPIO_PIN_RX);
    gpio_set_af(IBUS_USART_GPIO_PORT, IBUS_USART_AF, IBUS_USART_GPIO_PIN_RX);

    /* Enable USART interrupts. */
    nvic_enable_irq(IBUS_USART_NVIC);

    /* Set USART parameters to match iBus protocol. */
    usart_set_baudrate(IBUS_USART, 115200);
    usart_set_databits(IBUS_USART, 8);
    usart_set_stopbits(IBUS_USART, USART_STOPBITS_2);
    usart_set_mode(IBUS_USART, USART_MODE_RX);
    usart_set_parity(IBUS_USART, USART_PARITY_NONE);
    usart_set_flow_control(IBUS_USART, USART_FLOWCONTROL_NONE);
    usart_enable_rx_interrupt(IBUS_USART);
    usart_enable(IBUS_USART);

    initialized = true;
}

static struct rx_sample *ibus_recv(bool blocking)
{
    while ((blocking || !ibus_rcvd_first_packet) && !ibus_data_ready) {
        ;
    }

    ibus_data_ready = false;

    return &ibus_data;
}

static uint32_t ibus_time_last_rcvd_packet(void)
{
    if (!ibus_rcvd_first_packet) {
        return 0;
    }
    return time_last_rcvd_packet;
}
