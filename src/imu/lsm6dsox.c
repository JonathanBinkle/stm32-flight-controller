#include "imu/lsm6dsox.h"
#include "constants.h"
#include "imu/imu_driver.h"
#include "timer.h"
#include "util.h"
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define LSM6DSOX_BUS_ADDRESS 0x6a
#define LSM6DSOX_WHOAMI_ID 0x6c

/* I2C connection to LSM6DSOX. */
#define I2C_GPIO_PORT GPIOB
#define I2C_GPIO_PINS (GPIO6 | GPIO7)
#define I2C_GPIO_AF_NUM GPIO_AF4
#define I2C_GPIO_RCC RCC_GPIOB
#define I2C I2C1
#define I2C_RCC RCC_I2C1
#define I2C_SPEED i2c_speed_fm_400k
#define I2C_RST RST_I2C1

/* SPI connection to LSM6DSOX. We drive CS pin (PB12) manually.*/
#define SPI_GPIO_RCC RCC_GPIOB
#define SPI_GPIO_PORT GPIOB
#define SPI_GPIO_AF GPIO_AF5
#define SPI_GPIO_PINS (GPIO13 | GPIO14 | GPIO15)
#define SPI_GPIO_CS_PIN GPIO12
#define SPI SPI2_BASE
#define SPI_RCC RCC_SPI2
#define SPI_READ_MASK (1u << 7)
#define SPI_WRITE_MASK (~(1u << 7))

/* Data-ready interrupts from LSM6DSOX on PB6,7. */
#define IRQ_GPIO_PORT GPIOB
#define IRQ_GPIO_PORT_RCC RCC_GPIOB
#define IRQ_GPIO_PIN_GYRO GPIO6
#define IRQ_GPIO_PIN_ACC GPIO7
#define IRQ_EXTI_GYRO EXTI6
#define IRQ_EXTI_ACC EXTI7
#define IRQ_ISR exti9_5_isr
#define IRQ_NVIC NVIC_EXTI9_5_IRQ

/* Relevant LSM6DSOX register addresses. */
#define REG_COUNTER_BDR_REG1 0xb
#define REG_INT1_CTRL 0xd
#define REG_INT2_CTRL 0xe
#define REG_WHO_AM_I 0xf
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_CTRL4_C 0x13
#define REG_CTRL6_C 0x15
#define REG_CTRL8_XL 0x17
#define REG_CTRL9_XL 0x18
#define REG_STATUS_REG 0x1e
#define REG_OUTX_L_G 0x22

/* Output data rate. */
#define ODR_1660Hz 0b1000

/* Full scale selection (DS12814 Table 52 & 178) and sensitivity (Table 2):
 * - acceleromter: 16g
 * - gyroscope: 2000dps */
#define ACC_FSS 0b01
#define ACC_SENSITIVITY 0.488f
#define GYRO_FSS 0b110
#define GYRO_SENSITIVITY 70u

/* Physical connection to LSM6DSOX. */
struct lsm6dsox_conn {
    /* Setup the connection. */
    void (*setup)(void);

    /* Write DATA to IMU REGister. */
    void (*write_byte)(uint8_t reg, uint8_t data);

    /* Read N bytes from IMU REGister to buffer DST. */
    void (*read)(uint8_t reg, void *dst, size_t n);

    /* Read and return one byte from IMU REGister to buffer DST. */
    uint8_t (*read_byte)(uint8_t reg);
};

/* IMU measurement in raw units. Do NOT change order (breaks fetch_raw_data)! */
struct imu_raw {
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
};

static struct imu_raw latest_raw;
static struct imu_si latest_si;
static volatile bool initialized, gyro_drdy, acc_drdy;
static volatile uint32_t gyro_drdy_timestamp;

static void setup(void);
static void gyro_setup(uint8_t odr, uint8_t fs);
static void acc_setup(uint8_t odr, uint8_t fs);
static void fetch_raw_data(void);
static struct imu_si *get_or_fetch_si(void);
static uint32_t data_ready(void);

static void lsm6dsox_i2c_setup(void);
static void lsm6dsox_i2c_read(uint8_t reg, void *dst, size_t n);
static uint8_t lsm6dsox_i2c_read_byte(uint8_t reg);
static void lsm6dsox_i2c_write_byte(uint8_t reg, uint8_t data);

static void lsm6dsox_spi_setup(void);
static void lsm6dsox_spi_write_byte(uint8_t reg, uint8_t data);
static void lsm6dsox_spi_read(uint8_t reg, void *dst, size_t n);
static uint8_t lsm6dsox_spi_read_byte(uint8_t reg);

const struct imu_driver lsm6dsox_driver = {.imu_setup = setup,
                                           .get_or_fetch_si = get_or_fetch_si,
                                           .data_ready = data_ready};

static struct lsm6dsox_conn lsm6dsox_conn_i2c = {
    .setup = lsm6dsox_i2c_setup,
    .write_byte = lsm6dsox_i2c_write_byte,
    .read = lsm6dsox_i2c_read,
    .read_byte = lsm6dsox_i2c_read_byte};

static struct lsm6dsox_conn lsm6dsox_conn_spi = {
    .setup = lsm6dsox_spi_setup,
    .write_byte = lsm6dsox_spi_write_byte,
    .read = lsm6dsox_spi_read,
    .read_byte = lsm6dsox_spi_read_byte};

static struct lsm6dsox_conn lsm6dsox_conn;

static void lsm6dsox_i2c_setup(void)
{
    /* Setup GPIO for I2C. */
    rcc_periph_clock_enable(I2C_GPIO_RCC);
    gpio_mode_setup(I2C_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, I2C_GPIO_PINS);
    gpio_set_af(I2C_GPIO_PORT, I2C_GPIO_AF_NUM, I2C_GPIO_PINS);

    /* Setup I2C. */
    rcc_periph_clock_enable(I2C_RCC);
    i2c_peripheral_disable(I2C);
    i2c_set_speed(I2C, I2C_SPEED, rcc_apb1_frequency / 1e6);
    i2c_peripheral_enable(I2C);
}

static inline void lsm6dsox_i2c_read(uint8_t reg, void *dst, size_t n)
{
    cm_disable_interrupts();
    i2c_transfer7(I2C, LSM6DSOX_BUS_ADDRESS, &reg, 1, (uint8_t *)dst, n);
    cm_enable_interrupts();
}

static uint8_t lsm6dsox_i2c_read_byte(uint8_t reg)
{
    uint8_t res = 0;
    lsm6dsox_i2c_read(reg, &res, 1);
    return res;
}

static void lsm6dsox_i2c_write_byte(uint8_t reg, uint8_t data)
{
    uint8_t cmd[2] = {reg, data};
    cm_disable_interrupts();
    i2c_transfer7(I2C, LSM6DSOX_BUS_ADDRESS, cmd, 2, NULL, 0);
    cm_enable_interrupts();
}

static void lsm6dsox_spi_setup(void)
{
    /* Setup GPIO for SPI2. Drive CS pin manually instead of SPIx_NSS. */
    rcc_periph_clock_enable(SPI_GPIO_RCC);
    gpio_mode_setup(SPI_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_GPIO_PINS);
    gpio_set_af(SPI_GPIO_PORT, SPI_GPIO_AF, SPI_GPIO_PINS);
    gpio_mode_setup(SPI_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    SPI_GPIO_CS_PIN);
    gpio_set(SPI_GPIO_PORT, SPI_GPIO_CS_PIN);

    /* Setup SPI2. */
    rcc_periph_clock_enable(SPI_RCC);
    spi_disable(SPI);
    spi_init_master(
        SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    spi_enable_software_slave_management(SPI);
    spi_set_nss_high(SPI);

    spi_enable(SPI);
}

static void lsm6dsox_spi_read(uint8_t reg, void *dst, size_t n)
{
    uint8_t *buf = dst;

    gpio_clear(SPI_GPIO_PORT, SPI_GPIO_CS_PIN);

    /* Read starting at reg. Since we've set IF_INC in CTRL3_C, we get the next
     * N bytes without having to increment addresses manually. */
    spi_xfer(SPI, reg | SPI_READ_MASK);

    for (size_t i = 0; i < n; i++) {
        buf[i] = spi_xfer(SPI, 0);
    }

    gpio_set(SPI_GPIO_PORT, SPI_GPIO_CS_PIN);
}

static uint8_t lsm6dsox_spi_read_byte(uint8_t reg)
{
    uint8_t data;
    gpio_clear(SPI_GPIO_PORT, SPI_GPIO_CS_PIN);
    spi_xfer(SPI, reg | SPI_READ_MASK);
    data = spi_xfer(SPI, 0);
    gpio_set(SPI_GPIO_PORT, SPI_GPIO_CS_PIN);
    return data;
}

static void lsm6dsox_spi_write_byte(uint8_t reg, uint8_t data)
{
    gpio_clear(SPI_GPIO_PORT, SPI_GPIO_CS_PIN);
    spi_xfer(SPI, reg & SPI_WRITE_MASK);
    spi_xfer(SPI, data);
    gpio_set(SPI_GPIO_PORT, SPI_GPIO_CS_PIN);
}

static void fetch_raw_data(void)
{
    /* 12 addresses starting at REG_OUTX_L_G are data regs of gyro and acc. */
    lsm6dsox_conn.read(REG_OUTX_L_G, (uint8_t *)&(latest_raw), 12);

    /* Adjust signs to match my convention:
     * - accelerometer:
     *      - move up => positive az
     *      - move right => positive ay
     *      - move forward => positive ax
     * - gyroscope:
     *      - pitch forward => positive gy
     *      - roll right => positve gx
     *      - yaw clockwise => positive gz */
    latest_raw.ax = -latest_raw.ax;
    latest_raw.gx = -latest_raw.gx;
    latest_raw.gy = -latest_raw.gy;
    latest_raw.gz = -latest_raw.gz;

    gyro_drdy = false;
    acc_drdy = false;
}

static struct imu_si *get_or_fetch_si(void)
{
    if (gyro_drdy && acc_drdy) {
        fetch_raw_data();
    }

    float gyro_raw2si = GYRO_SENSITIVITY / 1e3f * DEG2RAD;
    latest_si.gx = latest_raw.gx * gyro_raw2si;
    latest_si.gy = latest_raw.gy * gyro_raw2si;
    latest_si.gz = latest_raw.gz * gyro_raw2si;

    float acc_raw2si = ACC_SENSITIVITY / 1e3f * GRAVITY_EARTH;
    latest_si.ax = latest_raw.ax * acc_raw2si;
    latest_si.ay = latest_raw.ay * acc_raw2si;
    latest_si.az = latest_raw.az * acc_raw2si;

    return &latest_si;
}

static uint32_t data_ready(void)
{
    if (gyro_drdy && acc_drdy) {
        return gyro_drdy_timestamp;
    }
    return 0u;
}

void IRQ_ISR(void)
{
    if (!initialized) {
        exti_reset_request(IRQ_EXTI_GYRO);
        exti_reset_request(IRQ_EXTI_ACC);
        return;
    }

    if (exti_get_flag_status(IRQ_EXTI_GYRO)) {
        exti_reset_request(IRQ_EXTI_GYRO);
        gyro_drdy = true;

        /* Remember when gyro data arrived for accurate `dt`. */
        gyro_drdy_timestamp = timer_now_us();
    }

    if (exti_get_flag_status(IRQ_EXTI_ACC)) {
        exti_reset_request(IRQ_EXTI_ACC);
        acc_drdy = true;
    }
}

static void gyro_setup(uint8_t odr, uint8_t fs)
{
    /* Set output data rate, full scale. */
    lsm6dsox_conn.write_byte(REG_CTRL2_G, (odr << 4) | (fs << 1));

    /* Low pass filter. See DS12814 Table 67 for bandwidth selection. */
    lsm6dsox_conn.write_byte(REG_CTRL4_C, (1 << 1));
    lsm6dsox_conn.write_byte(REG_CTRL6_C, 0b100);
}

static void acc_setup(uint8_t odr, uint8_t fs)
{
    /* Set output data rate, full scale, enable low pass filter. */
    lsm6dsox_conn.write_byte(REG_CTRL1_XL, (odr << 4) | (fs << 2) | (1 << 1));

    /* Low pass filter: bandwidth + fast settling mode. */
    lsm6dsox_conn.write_byte(REG_CTRL8_XL, (0b100 << 5) | (1 << 3));

    /* Disable I3C interface. */
    lsm6dsox_conn.write_byte(REG_CTRL9_XL,
                             (1 << 1) | /* Default: */ 0b11100000);
}

static void setup(void)
{
    if (initialized) {
        return;
    }
    gyro_drdy = false;
    acc_drdy = false;

    /* 10ms boot time (see AN5272 section 5.7). */
    timer_wait_us(1e4);

    /* Setup SPI bus. */
    lsm6dsox_conn = lsm6dsox_conn_spi;
    lsm6dsox_conn.setup();
    if (lsm6dsox_conn.read_byte(REG_WHO_AM_I) != LSM6DSOX_WHOAMI_ID) {
        error("[lsm6dsox] IMU ID doesn't match LSM6DSOX ID");
    }

    /* Reset IMU and wait until done. Set auto-increment. */
    lsm6dsox_conn.write_byte(REG_CTRL3_C, 1 | /* Default: */ 0b00000100);
    while (lsm6dsox_conn.read_byte(REG_CTRL3_C) & 1) {
        ;
    }

    /* Configure IMU output data rate, full scales, and low pass filters. */
    gyro_setup(ODR_1660Hz, GYRO_FSS);
    acc_setup(ODR_1660Hz, ACC_FSS);

    /* Create an interrupt when data is ready, even if the previous sample has
     * not been read yet ("dataready pulsed mode"). */
    lsm6dsox_conn.write_byte(REG_COUNTER_BDR_REG1, (1 << 7));

    /* Enable gyro interrupt on INT1, acc interrupt on INT2 of IMU. */
    lsm6dsox_conn.write_byte(REG_INT1_CTRL, (1 << 1));
    lsm6dsox_conn.write_byte(REG_INT2_CTRL, (1 << 0));
    fetch_raw_data(); /* Clear drdy flags. */

    /* Enable interrupts on STM32F411. */
    rcc_periph_clock_enable(IRQ_GPIO_PORT_RCC);
    rcc_periph_clock_enable(RCC_SYSCFG);
    gpio_mode_setup(IRQ_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
                    IRQ_GPIO_PIN_ACC | IRQ_GPIO_PIN_GYRO);
    exti_select_source(IRQ_EXTI_GYRO, IRQ_GPIO_PORT);
    exti_set_trigger(IRQ_EXTI_GYRO, EXTI_TRIGGER_RISING);
    exti_enable_request(IRQ_EXTI_GYRO);
    exti_select_source(IRQ_EXTI_ACC, IRQ_GPIO_PORT);
    exti_set_trigger(IRQ_EXTI_ACC, EXTI_TRIGGER_RISING);
    exti_enable_request(IRQ_EXTI_ACC);
    nvic_enable_irq(IRQ_NVIC);

    initialized = true;
}
