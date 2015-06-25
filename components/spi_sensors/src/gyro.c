/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

/*
 * Driver for chip L3GD20H
 */

/* standard */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include <gyro.h>
#include <spi_registers.h>
#include <spi_sensors_inf.h>
#include <spi_inf.h>

/* application common */
#include "common.h"
#include <periph.h>
#include <utils.h>

//#define DEBUG_GYRO
#ifdef DEBUG_GYRO
#define DGYRO(args...) \
    do { \
        printf("GYRO %s(%d):", __func__, __LINE__); \
        printf(args); \
        printf("\n"); \
    } while(0)
#else
#define DGYRO(...) do{}while(0)
#endif

#define WHO_AM_I 0xD4
#define READ_MASK 0x80
#define AUTO_INDEX 0x40

// Device registers
#define WHO_AM_I_REG 0xF
#define CTRL_REG1 0x20
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define OUT_X_L 0B00101000
#define OUT_X_H 0B00101001
#define OUT_Y_L 0B00101010
#define OUT_Y_H 0B00101011
#define OUT_Z_L 0B00101100
#define OUT_Z_H 0B00101101

#define GYRO_SCALE_245 133.7
#define GYRO_SCALE_500 65.54

static gyro_val_t cal;
static gyro_val_t filt[5] = {0, 0, 0, 0, 0};

void gyro_read(gyro_val_t *g)
{
    write_byte(OUT_X_L | READ_MASK | AUTO_INDEX);
    int low_x_i = read_byte();
    int high_x_i = read_byte();
    int low_y_i = read_byte();
    int high_y_i = read_byte();
    int low_z_i = read_byte();
    int high_z_i = read_byte();
    transfer(GYRO_ID);
    
    int16_t x = ((get_data(high_x_i) << 8) + get_data(low_x_i));
    int16_t y = ((get_data(high_y_i) << 8) + get_data(low_y_i));
    int16_t z = ((get_data(high_z_i) << 8) + get_data(low_z_i));
   
    g->y = ((double)x / GYRO_SCALE_500);// - cal.x;
    g->x = ((double)y / GYRO_SCALE_500);// - cal.y;
    g->z = ((double)z / GYRO_SCALE_500);// - cal.z;
    //return;
    // Filter values
    // Assume ~100Hz sample rate, 1st order band pass chebyshev filter
    // Using 10Hz and 0.01Hz cutoff, assuming vibrations any faster than that are gyro noise
    filt[0] = filt[1];
    filt[1] = filt[2];
    filt[2] = filt[3];
    filt[3] = filt[4];
    double c1 = 0.06802597496317;
    double c2 = -0.4164061293;
    double c3 = 1.9798711432;
    double c4 = -3.7081541503;
    double c5 = 3.1446784571;

    filt[4].x = (c1 * g->x) + (c2 * filt[0].x) + (c3 * filt[1].x) + (c4 * filt[2].x)+ (c5 * filt[3].x);
    g->x = (filt[0].x + filt[4].x) - 2 * filt[2].x;
    filt[4].y = (c1 * g->y) + (c2 * filt[0].y) + (c3 * filt[1].y) + (c4 * filt[2].y)+ (c5 * filt[3].y);
    g->y = (filt[0].y + filt[4].y) - 2 * filt[2].y;
    filt[4].z = (c1 * g->z) + (c2 * filt[0].z) + (c3 * filt[1].z) + (c4 * filt[2].z)+ (c5 * filt[3].z);
    g->z = (filt[0].z + filt[4].z) - 2 * filt[2].z;
}

void
gyro_init(void)
{
    DGYRO("Staring gyro init...");

    write_register(CTRL_REG1, 0xF); // Turn on all axis
	transfer(GYRO_ID);
    //write_register(CTRL_REG4, 1 << 4); // 500dps
	//transfer(GYRO_ID);
    udelay(1000);

    int ret_i = read_register(WHO_AM_I_REG);
    transfer(GYRO_ID);
    int ret = get_data(ret_i);

    char *buffer[100];

	if (ret != WHO_AM_I) {
        // NOTE: This always seems to be wrong but the gyro does
        // return values.
		//sprintf(buffer,"ERROR Wrong gyro WHO_AM_I, got %d instead!", ret);
	} else {
		//sprintf(buffer,"Correct gyro who am i: %d...!\n", ret);
	}
    //spi_log(buffer);
    //udelay(2000);
    gyro_read(&cal);
    //sprintf(buffer, "Calibration gyro values x:%lf y:%lf z:%lf\n", cal.x, cal.y, cal.z);
    //spi_log(buffer);
}
