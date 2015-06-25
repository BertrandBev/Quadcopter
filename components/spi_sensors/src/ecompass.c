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
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <spi_registers.h>
#include <spi_sensors.h>
#include <ecompass.h>
#include <spi_inf.h>
/* application common */
#include "common.h"
#include <periph.h>
#include <utils.h>

//#define DEBUG_ECOMPASS
#ifdef DEBUG_ECOMPASS
#define DECO(args...) \
    do { \
        printf("ECOMPASS %s(%d):", __func__, __LINE__); \
        printf(args); \
        printf("\n"); \
    } while(0)
#else
#define DECO(...) do{}while(0)
#endif

#define WHO_AM_I 0x49
#define AUTO_INDEX 0x40
#define READ_MASK 0x80
#define WHO_AM_I_REG 0xF
#define CTRL_REG1 0x20
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define CTRL_REG7 0x26
#define OUT_X_L_M 0B00001000
#define OUT_X_H_M 0B00001001
#define OUT_Y_L_M 0B00001010
#define OUT_Y_H_M 0B00001011
#define OUT_Z_L_M 0B00001100
#define OUT_Z_H_M 0B00001101
#define OUT_X_L_A 0B00101000
#define OUT_X_H_A 0B00101001
#define OUT_Y_L_A 0B00101010
#define OUT_Y_H_A 0B00101011
#define OUT_Z_L_A 0B00101100
#define OUT_Z_H_A 0B00101101

// Accelerometer scale
#define LA_SO_2G 16384

// Magnometer scale
#define M_SO_4GAUSS 8192

static ecompass_val_t cal;

void ecompass_read(ecompass_val_t *m)
{
    // Write the address to start reading from
    write_byte(OUT_X_L_M | READ_MASK | AUTO_INDEX);
    // Read bytes out in register order
    int low_m_x_i = read_byte();
    int high_m_x_i = read_byte();
    int low_m_y_i = read_byte();
    int high_m_y_i = read_byte();
    int low_m_z_i = read_byte();
    int high_m_z_i = read_byte();
    transfer(ECOMPASS_ID);
    int16_t x = ((get_data(high_m_x_i) << 8) + get_data(low_m_x_i));
    int16_t y = ((get_data(high_m_y_i) << 8) + get_data(low_m_y_i));
    int16_t z = ((get_data(high_m_z_i) << 8) + get_data(low_m_z_i));
    m->mag.x = ((double)x / M_SO_4GAUSS);
    m->mag.y = ((double)y / M_SO_4GAUSS);
    m->mag.z = ((double)z / M_SO_4GAUSS);
   
    // TODO: Its probably more efficient to read the registers inbetween the
    // sensor data registers to remove a transfer IPC call.  
    // Write the address to start reading from
    write_byte(OUT_X_L_A | READ_MASK | AUTO_INDEX);
    //Read bytes out in register order
    int low_a_x_i = read_byte();
    int high_a_x_i = read_byte();
    int low_a_y_i = read_byte();
    int high_a_y_i = read_byte();
    int low_a_z_i = read_byte();
    int high_a_z_i = read_byte();
    transfer(ECOMPASS_ID);
    x = ((get_data(high_a_x_i) << 8) + get_data(low_a_x_i));
    y = ((get_data(high_a_y_i) << 8) + get_data(low_a_y_i));
    z = ((get_data(high_a_z_i) << 8) + get_data(low_a_z_i));
    m->acc.x = ((double)x / LA_SO_2G);
    m->acc.y = ((double)y / LA_SO_2G);
    m->acc.z = ((double)z / LA_SO_2G);
}

void
calibrate() {
    // Magnometer readings should form a sphere
    // There are 2 types of errors that need to be corrected
    // Hard iron - offset of the sphere
    // Soft iron - distortion of sphere

    // Hard iron: Rotate the quadcopter and get max and min values of each axis
    // The average of max and min is the offset

    // Soft iron: Find the minor and major axis of the sphere
    // Rotate quadcopter and get the magnitude of each point. If data is clean then
    // the largest magnitude will be the major axis, smallest the minor axis. Then
    // find the rotation matrix to align sphere with global frame.
    // Then get scale factor for each axis, apply it then rotate reading back.

    // Rotate the quadcopter and output data points
    ecompass_val_t v[100];
    int i;
    for (i = 0; i < 100; i++) {
        ecompass_read(&(v[i]));
        udelay(100000);
    }
} 

void
ecompass_init(void)
{
    DECO("Starting ecompass init...");
    write_register(CTRL_REG1, 0x37);
    transfer(ECOMPASS_ID);
    // Acceleromter data ready on int1
    //write_register(CTRL_REG3, 1 << 2); 
    // Magnometer data ready on int2
    //write_register(CTRL_REG4, 1 << 2);

    write_register(CTRL_REG5, 0x8);
    transfer(ECOMPASS_ID);
    write_register(CTRL_REG7, 0x0);
    transfer(ECOMPASS_ID);
	udelay(400000);

    int ret;
    int ret_i = read_register(WHO_AM_I_REG | READ_MASK);
    transfer(ECOMPASS_ID); 
    if ((ret = get_data(ret_i)) != WHO_AM_I) {
        DECO("E-Comapss initilisation failed (who_am_i is %d)\n", ret);
    } else {
        DECO("Init success!\n");
    }

    //udelay(500000);
    //calibrate();
}
