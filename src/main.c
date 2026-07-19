#include "esc/esc.h"
#include "esc/oneshot.h"
#include "esc/pwm.h"
#include "fc.h"
#include "imu/imu_frontend.h"
#include "imu/lsm6dsox.h"
#include "led.h"
#include "mm.h"
#include "rx/ibus.h"
#include "rx/rx.h"
#include "telemetry/telemetry.h"
#include "telemetry/usb_telemetry.h"
#include "timer.h"
#include <libopencm3/stm32/rcc.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAIN_LOOP_PERIOD_US 2000u /* 500 Hz */

static struct imu_driver imu_driver;
static struct rx_driver rx_driver;
static struct telemetry_backend telemetry;
static struct esc_pwm_driver esc_driver;

static struct rx_sample rx_latest;
static struct imu_sample imu_latest;
static struct imu_angles imu_angles;
static struct esc_throttles throttles;
static struct fc_out fc_out;

static void blink(void)
{
    for (int i = 0; i < 20; i++) {
        led_toggle();
        timer_wait_us(1e5);
    }
}

int main(void)
{
    /* Clock setup.
     * Use 25 MHz HSE crystal as PLL input.
     * Configure the PLL to output a 84 MHz SYSCLK and 48 MHz USB clock. */
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);

    /* Setup modules. */
    timer_setup();
    led_setup();

    telemetry = usb_telemetry;
    telemetry.setup();

    rx_driver = ibus_driver;
    rx_driver.rx_setup();

    imu_driver = lsm6dsox_driver;
    imu_driver.imu_setup();
    imu_calibrate(&imu_driver, true);

    esc_driver.base = &pwm_driver;
    esc_driver.pwm_config = oneshot42;
    esc_driver.base->esc_setup(&esc_driver);

    fc_setup(MODE_STABILIZE);

    blink(); /* Setup done visualizer. */

    uint32_t next_iteration_start_us = timer_now_us();
    while (true) {
        timer_wait_until_us(next_iteration_start_us);
        next_iteration_start_us += MAIN_LOOP_PERIOD_US;

        rx_latest = *rx_driver.rx_recv(false);

        imu_latest = *imu_get_latest(&imu_driver);

        imu_angles = *update_complementary_filter(&imu_latest);

        fc_out =
            *fc_update(&rx_latest, &imu_angles, &imu_latest.si, timer_now_us());

        if (rx_driver.rx_time_last_rcvd_packet() > 0 &&
            timer_now_us() - rx_driver.rx_time_last_rcvd_packet() > 500000) {
            /* No RX packet received within 500ms. Classify as RX signal loss.
             * Disarm motors for safety (until RX signal is healthy again). */
            throttles.front_left = ESC_MIN_THROTTLE;
            throttles.front_right = ESC_MIN_THROTTLE;
            throttles.back_left = ESC_MIN_THROTTLE;
            throttles.back_right = ESC_MIN_THROTTLE;
        } else {
            throttles = *motor_mixing(fc_out.roll_out, fc_out.pitch_out,
                                      fc_out.yaw_out, &rx_latest);
        }

        esc_driver.base->esc_update(&esc_driver, &throttles);

        /* Send telemetry data. */
        telemetry_tick();
        uint32_t deadline_us = next_iteration_start_us - /* margin: */ 20;
        telemetry.send(RX_SAMPLE, &rx_latest, sizeof rx_latest, deadline_us);
        telemetry.send(IMU_SI, &imu_latest.si, sizeof imu_latest.si,
                       deadline_us);
        telemetry.send(IMU_ANGLES, &imu_angles, sizeof imu_angles, deadline_us);
        telemetry.send(IMU_BIAS, imu_get_bias(), sizeof(struct imu_bias),
                       deadline_us);
        telemetry.send(FC_OUT, &fc_out, sizeof fc_out, deadline_us);
        telemetry.send(PID_PITCH, fc_get_pid(PITCH), sizeof(struct pid),
                       deadline_us);
        telemetry.send(PID_ROLL, fc_get_pid(ROLL), sizeof(struct pid),
                       deadline_us);
        telemetry.send(PID_YAW, fc_get_pid(YAW), sizeof(struct pid),
                       deadline_us);
        telemetry.send(ESC_THROTTLES, &throttles, sizeof throttles,
                       deadline_us);
    }
}
