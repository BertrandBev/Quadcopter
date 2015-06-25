/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */
#include <i2c.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <utils.h>
#include <assert.h>
#include <platsupport/i2c.h>
#include <common.h>
#include <i2c_inf.h>

//#define DEBUG_I2C
#ifdef DEBUG_I2C
#define DI2C(args...) \
    do { \
        printf("I2C %s(%d): ", __func__, __LINE__); \
        printf(args); \
        printf("\n"); \
    } while(0)
#else
#define DI2C(...) do{}while(0)
#endif

// Uncomment to print what the i2c driver is writing to the bus
//#define PRINT_OUTPUT 

mux_sys_t exynos_mux;
i2c_bus_t i2c_bus;
i2c_slave_t i2c_pwm;
// Used by the I2C callback to signal when the transaction is complete 
int finished; 

struct i2c_device_config {
    int address;
    i2c_slave_t slave;
};

/**
 * Device parameters. An address in the table should match the address of 
 * an incoming request 
 */
#define DEVICE_PARAMS(a)                \
        {                               \
            .address = a,               \
        }

static struct i2c_device_config device_params[] = {
    DEVICE_PARAMS(PWM_ADDR), 
};

struct i2c_client_config {
    int id;
    i2c_dev_port_t **port;
};

/**
 * Client parameters. An id in the table should match the app id of 
 * an incoming request 
 */
#define CLIENT_PARAMS(a, p)             \
        {                               \
            .id = a,                    \
            .port = (i2c_dev_port_t**)p  \
        }

static struct i2c_client_config client_params[] = {
    CLIENT_PARAMS(MOTOR_APP_ID, &i2c_motors), 
};

/**
 * Pulls the device configuration from the database
 */
static struct i2c_device_config*
get_device_config(int addr)
{
    int i;
    for(i = 0; i < ARRAY_SIZE(device_params); i++){
        if(device_params[i].address == addr){
            return &device_params[i];
        }
    }
    return NULL;
}

/**
 * Pulls the client configuration from the database
 */
static struct i2c_client_config*
get_client_config(int id)
{
    int i;
    for(i = 0; i < ARRAY_SIZE(client_params); i++) {
        if(client_params[i].id == id){
            return &client_params[i];
        }
    }
    return NULL;
}

/**
 * Called on every I2C interrupt, direct control to driver.
 * This is in the same thread as the wait call.
 */
void
i2c_irq_event(void *arg)
{
    (void)arg;
    DI2C(".");
    i2c_handle_irq(&i2c_bus);
    i2c0_int_reg_callback(&i2c_irq_event, NULL);
}

static void
i2c_complete_cb(i2c_bus_t *bus, enum i2c_stat status, size_t size, void* token) {
    DI2C("Got callback\n");
    finished = TRUE;
}

// Does the writes then the reads.
// The idea is to be able to batch register requests together
// If large numbers of interleaved reads/writes are required then a
// new interface is needed.
// App_id is only needed until badge support in camkes
int i2c_transfer(int app_id, int dev_addr, unsigned int wcount, unsigned int rcount)
{
    DI2C("Got request. Write %d, read %d", wcount, rcount);
    struct i2c_client_config *cfg = get_client_config(app_id);
    struct i2c_device_config *dfg = get_device_config(dev_addr);
    assert(cfg);
    assert(dfg);
    if (!cfg || !dfg) {
        return -1;
    }

    DI2C("Doing write");
    // Do the writes
#ifdef PRINT_OUTPUT
    int i;
    for (i = 0; i < wcount; i++) {
        printf("%x ", (*(cfg->port))->txbuf[i]);
    }
    printf("\n");
#endif
    int count = i2c_slave_write(&(dfg->slave), (*(cfg->port))->txbuf, wcount, &i2c_complete_cb, NULL);
    if (count >= 0) {
        finished = 0;
        while (!finished) {
            i2c0_int_wait();
        }
    } else {
        return -1;
    } 

    if (rcount == 0) return 0;

    DI2C("Doing read");
    // Do the reads
    count = i2c_slave_read(&(dfg->slave), (*(cfg->port))->rxbuf, rcount, &i2c_complete_cb, NULL);
    if (count >= 0) {
        finished = 0;
        while (!finished) {
            i2c0_int_wait();
        }
    } else {
        return -1;
    }
    return 0;
} 

void i2c__init(void)
{
    DI2C("Starting I2C driver");
    exynos_mux_init(gpio1base_i2c, gpio2base_i2c, NULL, NULL, &exynos_mux);
    DI2C("Enabled mux\n");
    int err = exynos_i2c_init(I2C0, i2c0, &exynos_mux, &i2c_bus);  
    assert(!err);
    
    // Initalise slaves
    int i;
    for (i = 0; i < ARRAY_SIZE(device_params); ++i) {
        err = i2c_kvslave_init(&i2c_bus, device_params[i].address, BIG8, BIG8, 
                &(device_params[i].slave));
        assert(!err);
    }

#ifdef DEBUG_I2C
    // Scan the bus for the pwm chip
    int count, addr;
    int start = 0;
    int found = 0;
    do {
        count = i2c_scan(&i2c_bus, start, &addr, 1);
        if(count){
            DI2C("Slave @ 0x%x\n", addr);
            if(addr == PWM_ADDR){
                found = 1;
                break;
            }
        }
    } while(count);
    // Check that the address responded  
    if (found) {
        DI2C("Received response from PWM address\n");
    } else {
        DI2C("No reponse from PWM\n");
    } 
#endif

    i2c0_int_reg_callback(&i2c_irq_event, NULL);
    DI2C("Finished I2C init");
}

