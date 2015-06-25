/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <spi.h>
#include <spi_inf.h>
#include <platsupport/spi.h>
#include <platsupport/gpio.h>
#include <utils.h>
#include <string.h>
#include <errno.h>
#include <platsupport/clock.h>
#include <common.h>
#include <platsupport/timer.h>
#include <platsupport/plat/timer.h>

#define SPI_PORT          SPI1
#define SPI_SPEED_DEFAULT 10000000

#define SPI_CS_RELEASE 1
#define SPI_CS_ASSERT  0

//#define DEBUG_SPI
#ifdef DEBUG_SPI
#define DSPI(args...) \
    do { \
        printf("SPI %s(%d): ", __func__, __LINE__); \
        printf(args); \
        printf("\n"); \
    } while(0)
#else
#define DSPI(...) do{}while(0)
#endif

static pstimer_t *timer;

struct spi_device_config {
    int id;
    int speed_hz;
    int nss_usdelay;
    void (*cs)(int state);
};

/**
 * Device parameters. An ID in the table should match the id of an incoming request 
 */
#define DEVICE_PARAMS(i, s, d, g)     \
        {                               \
            .id = i,                    \
            .speed_hz = s,              \
            .nss_usdelay = d,           \
            .cs = g                     \
        }

const static struct spi_device_config device_params[] = {
    DEVICE_PARAMS(MPU_ID ,    800000,  0, &gpio_spi_mpu_nss), // Max clk frequency is 1 MHz
    DEVICE_PARAMS(ECOMPASS_ID, 800000, 0, &gpio_spi_acc_mag_nss),
    DEVICE_PARAMS(GYRO_ID,    800000,  0, &gpio_spi_gyro_nss),
    DEVICE_PARAMS(BARO_ID,    10000000, 0, &gpio_spi_baro_nss),
};

// TODO: Add something that deals with switching buffers for different clients.
// Not currently necessary since there is only one client and would be best done
// after badging end points is supported in camkes.

/// A handle to the SPI bus that this component drives
static spi_bus_t* spi_bus;
/// The current speed of the bus to avoid unnecessary recalibration
static long spi_cur_speed;
int finished;
clock_sys_t clock_sys;

unsigned int clktree_set_spi1_freq(unsigned int rate){
    clk_t* clk;
    clk = clk_get_clock(&clock_sys, CLK_SPI1);
    return clk_set_freq(clk, rate);
}

/**
 * Pulls the device configuration from the database
 */
static const struct spi_device_config*
get_device_config(int id)
{
    int i;
    for(i = 0; i < ARRAY_SIZE(device_params); i++){
        if(device_params[i].id == id){
            return &device_params[i];
        }
    }
    return NULL;
}

static inline void
chip_select(const struct spi_device_config* cfg, int state)
{
    udelay(cfg->nss_usdelay);
    cfg->cs(state);
    udelay(cfg->nss_usdelay);
}

static inline void
set_speed(const struct spi_device_config* cfg)
{
    if(spi_cur_speed != cfg->speed_hz){
        clktree_set_spi1_freq(cfg->speed_hz);
        spi_cur_speed = cfg->speed_hz;
    }
}

/**
 * SPI driver calls this when the transfer is complete.
 * All we need to do is store the status and post on the semaphore
 * that the main thread is waiting on.
 */
uint64_t wait_time = 0;
uint64_t stamp, start;
static void
spi_complete_callback(spi_bus_t* bus, int status, void* token)
{
    DSPI("Finished transfer");
    *(int*)token = status;
    finished = TRUE;
}

/**
 * Called on every SPI IRQ. Redirect control to the driver.
 * TODO: Remove this when camkes can wait on internal async eps
 */
void
spi_irq_event(void *arg)
{
    (void)arg;
    spi_handle_irq(spi_bus);
    wait_time += timer->get_time(timer) - stamp;
    spi1_int_reg_callback(&spi_irq_event, NULL);
}

/* Camkes entry point */
void
spi__init(void)
{
    DSPI("Starting init");
    // Initalise the clock tree
    int err;
    err = exynos5_clock_sys_init(cmu_cpu_clk,
                                 cmu_core_clk,
                                 NULL,
                                 NULL,
                                 cmu_top_clk,
                                 NULL,
                                 NULL,
                                 NULL,
                                 NULL,
                                 NULL, // TODO: Check this is the right thing to pass into mem
                                 &clock_sys);
    assert(!err);
    if(err){
        printf("Failed to initialise clock tree\n");
    }

    clktree_set_spi1_freq(SPI_SPEED_DEFAULT);
    spi_cur_speed = SPI_SPEED_DEFAULT;

    /* Initialise the SPI bus */
    err = exynos_spi_init(SPI_PORT, spi1_reg, NULL, NULL, &spi_bus); 
    assert(!err);
    if(err){
        LOG_ERROR("Failed to initialise SPI port\n");
        return;
    }
    
    timer = generic_timer_get_timer();

    /* Register an IRQ callback for the driver */
    spi1_int_reg_callback(&spi_irq_event, NULL);
    DSPI("Finished init");
}

/**
 * Performs an SPI transfer
 */
static int
do_spi_transfer(const struct spi_device_config* cfg, void* txbuf, unsigned int wcount, 
                void* rxbuf, unsigned int rcount)
{
    int ret;
    int status;
    set_speed(cfg);
    chip_select(cfg, SPI_CS_ASSERT);

    start = timer->get_time(timer);
    /* Begin the transfer */
    ret = spi_xfer(spi_bus, txbuf, wcount, rxbuf, rcount, spi_complete_callback, &status);
    wait_time = 0;
    if (ret >= 0) {
        finished = 0;
        while (!finished) {
            stamp = timer->get_time(timer);
            spi1_int_wait();
        }
        ret = status;
    }

    chip_select(cfg, SPI_CS_RELEASE);
    return ret;
}

/**
 * Initiate the transfer of shared data
 */
int
spi_transfer(int id, unsigned int wcount, unsigned int rcount)
{
    const struct spi_device_config* cfg;
    /* Find the device configuration */
    cfg = get_device_config(id);
    assert(cfg);
    if(cfg == NULL){
        return -1;
    }
    /* Transfer the data from the shared data port */
    // TODO: Change the port based on the badged client
    return do_spi_transfer(cfg, ((spi_dev_port_p)spi1_spi_sensors)->txbuf, wcount, 
            ((spi_dev_port_p)spi1_spi_sensors)->rxbuf, rcount);
}

