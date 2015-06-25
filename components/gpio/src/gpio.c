/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <platsupport/mux.h>
#include <platsupport/gpio.h>
#include <platsupport/irq_combiner.h>
#include <platsupport/timer.h>
#include <platsupport/plat/timer.h>
#include <platsupport/mach/pwm.h>
#include "utils.h"
#include <ppm_inf.h>

#include <gpio.h>

#define UART0_CTSN  GPIOID(GPA0, 2)
#define UART0_RTSN  GPIOID(GPA0, 3)

#define CAN_CSn     XEINT16
#define CAN_INTn    XEINT15
#define CAN_RESETn  XEINT25
#define MPU_CS      XEINT14
#define MPU_INT     XEINT8
#define ACC_MAG_CS  XEINT21
#define ACC_INT     XEINT18
#define MAG_INT     XEINT23
#define GYRO_CS     XEINT11
#define GYRO_INT    XEINT20
#define BARO_CS     XEINT10
#define SPI_EXT_CS  XEINT13
#define SPI_EXT_INT XEINT19
#define PPM_GPIO    XEINT5
#define PWM_EN      XEINT17
#define LIDAR_INT   UART0_CTSN

#define CAN_EINT_CIRQ      XEINT15_CIRQ
#define MPU_EINT_CIRQ      XEINT8_CIRQ

#define CAN_EINT_IRQ       63
#define MPU_EINT_IRQ       60
#define ACC_EINT_IRQ       64
#define MAG_EINT_IRQ       64
#define GYRO_EINT_IRQ      64
#define SPI_EXT_EINT_IRQ   64

#define PPM_IRQ            58
#define PPM_CIRQ           XEINT5_CIRQ

// PPM
#define MIN_CHANNELS 6

#define CS_FUNC(gpio)             \
    void                          \
gpio_##gpio(int state)        \
{                             \
    if(state)                 \
    gpio_set(&o_##gpio);  \
    else                      \
    gpio_clr(&o_##gpio);  \
}

//#define DEBUG_PPM
#ifdef DEBUG_PPM
#define DPPM(args...) \
    do { \
        printf("PPM %s(%d): ", __func__, __LINE__); \
        printf(args); \
        printf("\n"); \
    } while(0)
#else
#define DPPM(...) do{}while(0)
#endif

//#define DEBUG_GPIO
#ifdef DEBUG_GPIO
#define DGPIO(args...) \
    do { \
        printf("GPIO %s(%d): ", __func__, __LINE__); \
        printf(args); \
        printf("\n"); \
    } while(0)
#else
#define DGPIO(...) do{}while(0)
#endif

mux_sys_t exynos_mux;
gpio_sys_t gpio_sys;
irq_combiner_t irq_combiner;

/* SPI chip select */
gpio_t o_spi_can_nss;
gpio_t o_spi_mpu_nss;
gpio_t o_spi_acc_mag_nss;
gpio_t o_spi_gyro_nss;
gpio_t o_spi_baro_nss;
gpio_t o_spi_ext_nss;
CS_FUNC(spi_can_nss);
CS_FUNC(spi_mpu_nss);
CS_FUNC(spi_acc_mag_nss);
CS_FUNC(spi_gyro_nss);
CS_FUNC(spi_baro_nss);
CS_FUNC(spi_ext_nss);

/* PPM Input */
gpio_t i_ppm;

/* PWM Enable */
gpio_t o_pwm_en;
CS_FUNC(pwm_en);

// Blade interrupt
gpio_t i_spi_ext_int;

double ppm_buf[MAX_CHANNELS] = {-1};
ppm_channels_t *channels; 
static int buf_index;
static int valid_channels = 0;
int isValid = 0;
int numReadings = 0; // For debug print out throttling

uint64_t prev_time;
pstimer_t *timer;
pwm_config_t config;

// PPM Thread
static void
irq_grp26_event(void* arg) {
    if (gpio_is_pending(&i_ppm)) {
        gpio_pending_clear(&i_ppm);
        uint64_t cur_time = timer->get_time(timer);
        uint64_t diff_us = (cur_time - prev_time) / 1000;
        DPPM("%llu", (unsigned long long) diff_us);

        if (diff_us > START_FRAME_MIN_US) {
            // We found a start frame. Signals the correct time apart
            // will be valid after receiving the start frame.
            if (buf_index > 2) {
                // We've got some valid signals
                int i;
                for (i = 0; i < buf_index; i++)
                    channels->c[i] = ppm_buf[i];


                channels->num_valid = buf_index;
                valid_channels = buf_index;
            }
            buf_index = 0;
            isValid = 1;
        } else if (diff_us > MIN_DATA_WIDTH_US && diff_us < MAX_DATA_WIDTH_US && isValid) {
            ppm_buf[buf_index] = diff_us;
            buf_index++;
        } else
            isValid = 0;

        prev_time = cur_time;
    }
    irq_grp26_int_reg_callback(&irq_grp26_event, NULL);
}

static void
irq_xint16_31_event(void *arg)
{
    if(gpio_is_pending(&i_spi_ext_int)){
        gpio_pending_clear(&i_spi_ext_int);
        // Blade has been triggered
        DGPIO("Got blade interrupt");
        motors_quick_kill(); 
    }
    xint16_31_int_reg_callback(&irq_xint16_31_event, NULL);
}


void gpio__init(void) {
    DGPIO("Starting init");
    exynos_mux_init(gpio1base, gpio2base, NULL, NULL, &exynos_mux);
    exynos_gpio_sys_init(&exynos_mux, &gpio_sys);
    exynos_irq_combiner_init(irqcbase, &irq_combiner);

    /* Enable UART0, UART3 and SPI0. */
    // TODO: Get SPI component to enable it's own mux
    mux_feature_enable(&exynos_mux, MUX_UART0);
    mux_feature_enable(&exynos_mux, MUX_UART1);
    mux_feature_enable(&exynos_mux, MUX_UART3);
    mux_feature_enable(&exynos_mux, MUX_SPI1);

    /* SPI chip selects */
    gpio_new(&gpio_sys, CAN_CSn,    GPIO_DIR_OUT, &o_spi_can_nss);
    gpio_new(&gpio_sys, MPU_CS,     GPIO_DIR_OUT, &o_spi_mpu_nss);
    gpio_new(&gpio_sys, ACC_MAG_CS, GPIO_DIR_OUT, &o_spi_acc_mag_nss);
    gpio_new(&gpio_sys, GYRO_CS,    GPIO_DIR_OUT, &o_spi_gyro_nss);
    gpio_new(&gpio_sys, BARO_CS,    GPIO_DIR_OUT, &o_spi_baro_nss);
    gpio_set(&o_spi_can_nss);
    gpio_set(&o_spi_mpu_nss);
    gpio_set(&o_spi_acc_mag_nss);
    gpio_set(&o_spi_gyro_nss);
    gpio_set(&o_spi_baro_nss);

    /* Blade */
    gpio_new(&gpio_sys, SPI_EXT_CS, GPIO_DIR_OUT, &o_spi_ext_nss);
    gpio_new(&gpio_sys, SPI_EXT_INT, GPIO_DIR_IRQ_RISE, &i_spi_ext_int);
    gpio_clr(&o_spi_ext_nss);

    /* PPM */
    gpio_new(&gpio_sys, PPM_GPIO,    GPIO_DIR_IRQ_RISE, &i_ppm);
    timer = generic_timer_get_timer();
    channels = (ppm_channels_t*)ppm; // Shared dataport pointer
    channels->c[0] = 0.1;
    channels->c[1] = 0.2;
    channels->c[2] = 0.3;
    channels->c[3] = 0.4;

    /* PWM */
    gpio_new(&gpio_sys, PWM_EN, GPIO_DIR_OUT, &o_pwm_en);
    gpio_clr(&o_pwm_en); // Always enable

    /* Configure IRQs that appear on the combiner */
    irq_combiner_enable_irq(&irq_combiner, PPM_CIRQ);
    irq_grp26_int_reg_callback(&irq_grp26_event, NULL);
    xint16_31_int_reg_callback(&irq_xint16_31_event, NULL);
    DGPIO("Finished init");
}
