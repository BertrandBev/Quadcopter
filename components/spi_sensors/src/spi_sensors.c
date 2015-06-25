/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <spi_sensors.h>
#include <stdio.h>
#include <utils.h>
#include <assert.h>
#include <common.h>

#include <spi_inf.h>
#include <spi_sensors_inf.h>
#include <gyro.h>
#include <baro.h>
#include <mpu.h>
#include <ecompass.h>

//#define DEBUG_SPI_SENSORS
#ifdef DEBUG_SPI_SENSORS
#define DSPIS(args...) \
    do { \
        printf("SPI Sensors %s(%d): ", __func__, __LINE__); \
        printf(args); \
        printf("\n"); \
    } while(0)
#else
#define DSPIS(...) do{}while(0)
#endif
spi_dev_port_p spi_dev;

int wcount = 0;

void write_byte(int value)
{
    spi_dev->txbuf[wcount++] = value;
}

void write_register(int address, unsigned char value)
{
    write_byte(address);
    write_byte(value);
}

// We want to be able to do bulk reads / writes to avoid
// unnecessary context switches. Write out zeros instead of
// explicitcally doing a read and then read the rxbuf at the
// index given by this function after making the spi transfer.
int read_byte()
{
    spi_dev->txbuf[wcount] = 0x0;
    return wcount++;
}

int read_register(int address)
{
    write_byte(address);
    return read_byte();
}

int get_data(int index) {
    return spi_dev->rxbuf[index];
}

// Reset the buffer to the begining
void clear() {
    wcount = 0;
}

// Make the spi transfer, resets the buffer for the next bulk transfer
void transfer(int dev_id)
{
    spi_transfer(dev_id, wcount, 0);
    wcount = 0;
}


static sensors_t last_read;

sensors_t sensors_get_last_read()
{
    return last_read;
}

sensors_t sensors_read()
{
    DSPIS("Doing a read");
    sensors_t s;
    // Go through each sensor and add it's data to the struct
    //baro_read(&s.baro);
    ecompass_read(&s.ecompass);
    mpu_read(&s.mpu);
    gyro_read(&s.gyro);
    last_read = s;
    return s;
}

void sensors__init()
{
    DSPIS("SPI Sensor init start\n");
    spi_dev = (spi_dev_port_p) spi_buf;

    baro_init();
    ecompass_init();
    mpu_init();
    gyro_init();
}   
