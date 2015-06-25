/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

/* standard */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <baro.h>
#include <spi_inf.h>
#include <baro_inf.h>
#include <spi_registers.h>

/* application common */
#include "common.h"
#include <periph.h>
#include <utils.h>

//#define DEBUG_BARO
#ifdef DEBUG_BARO
#define DBARO(args...) \
    do { \
        printf("BARO %s(%d):", __func__, __LINE__); \
        printf(args); \
        printf("\n"); \
    } while(0)
#else
#define DBARO(...) do{}while(0)
#endif

// Device registers
#define RESET 0x1E
#define PROM_READ 0xA0
#define D1_REG 0x48
#define D2_REG 0x58

// mbar to m (linear approximation near sea level)
#define MBAR_TO_M 0.098684

#include <spi_sensors_inf.h>

// Calibration bits
unsigned int C[7];

static double initial_height = 0;

static void reset(void) 
{
    DBARO("Resetting");
    write_byte(RESET);
    transfer(BARO_ID);
    udelay(1000000);
}

static uint32_t read_PROM(int address)
{
    write_byte(address);
    int high_i = read_byte();
    int low_i = read_byte();
    transfer(BARO_ID);
	int high = get_data(high_i);
	int low = get_data(low_i);
    int ret = (high << 8) + low;
    DBARO("Got high %d, low %d", high, low);
    return ret;
}

void baro_init(void)
{
    reset();
    int i;
    // NOTE: These reads could be batched
    for (i = 1; i < 7; i++) {
        C[i] = read_PROM(PROM_READ | (i << 1));
        DBARO("C[%d]: %d\n", i, C[i]);
    }
    // Calibrate sensor
    baro_val_t b;
    baro_read(&b);
    initial_height = b.pressure;
}

static uint32_t read_ADC(int address)
{
    write_byte(address);
    transfer(BARO_ID);
    //udelay(10000); // Used to be 20ms
    DBARO("Waiting 10ms for ADC...");

    write_byte(0); // Command to read
    int high_i = read_byte();
    int mid_i =  read_byte();
    int low_i = read_byte();
    transfer(BARO_ID);
    int high = get_data(high_i);
    int mid = get_data(mid_i);
    int low = get_data(low_i);  
    DBARO("Got high %d, mid %d, low %d", high, mid, low);
    return (high << 16) + (mid << 8) + low;  
}

void baro_read(baro_val_t* b)
{
    uint32_t D1, D2;
    int64_t dT, off, sens, t, p;
    double T, P;
    // Can't batch these since we need to trigger the ADC conversion
    // then 10ms before reading values.
    D1 = read_ADC(D1_REG);
    D2 = read_ADC(D2_REG);
    DBARO("D1: %d, D2: %d\n", D1, D2);

    dT = D2 - (((uint32_t)C[5]) << 8);
    off  = ((int64_t)C[2] << 16) + ((dT * C[4]) >> 7);
    sens = ((int32_t)C[1] << 15) + ((dT * C[3]) >> 8);

    t = (((int64_t)dT * (int64_t)C[6]) / 8388608) + 2000;
    T = (double)t / 100; 
  
    p  = ((int64_t)D1 * sens / 2097152 - off) / 32768;
    P = (double)p / 100;

    b->temperture = T;
    // TODO: Rename pressure to height
    b->pressure = (P * MBAR_TO_M) - initial_height;
}
