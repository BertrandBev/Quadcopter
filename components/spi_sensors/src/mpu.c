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
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <mpu.h>
#include <spi_inf.h>
#include <spi_registers.h>
#include <spi_sensors_inf.h>
/* application common */
#include "common.h"
#include <periph.h>
#include <utils.h>

#define WHO_AM_I 0x68
#define READ_MASK 0x80

//#define DEBUG_MPU
#ifdef DEBUG_MPU
#define DMPU(args...) \
    do { \
        printf("MPU %s(%d):", __func__, __LINE__); \
        printf(args); \
        printf("\n"); \
    } while(0)
#else
#define DMPU(...) do{}while(0)
#endif

// Device registers
#define SELF_TEST_X 0xD
#define SELF_TEST_Y 0xE
#define SELF_TEST_Z 0xF
#define SELF_TEST_A 0x10
#define GYRO_CONFIG 0x1B
#define WHO_AM_I_REG 0x75
#define USER_CTRL 0x6A
#define SIGNAL_PATH_RESET 0x68
#define SAMPLE_RATE_DIV 0x19
#define PWR_MGMT_1 0x6B
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define INT_STATUS 0x3A
#define OUT_X_L_G 0x44
#define OUT_X_H_G 0x43
#define OUT_Y_L_G 0x46
#define OUT_Y_H_G 0x45
#define OUT_Z_L_G 0x48
#define OUT_Z_H_G 0x47
#define OUT_X_L_A 0x3C
#define OUT_X_H_A 0x3B
#define OUT_Y_L_A 0x3E
#define OUT_Y_H_A 0x3D
#define OUT_Z_L_A 0x40
#define OUT_Z_H_A 0x3F

// Accelerometer scale factor (per g)
#define AFS_SEL_0 16384
#define AFS_SEL_1 8192

// Gyro scale factor (per degree)
#define FS_SEL_0 131
static gyro_val_t filt[5] = {0, 0, 0, 0, 0};
static mpu_val_t cal;

static int mpu_read_register(int address)
{
    return read_register(READ_MASK | address);
}

void mpu_read(mpu_val_t *m)
{
    clear();
    write_byte(OUT_X_H_A | READ_MASK);
    int i;
    for (i = 0; i < 14; i++) read_byte();
    transfer(MPU_ID);

    int16_t x = ((get_data(9) << 8) + get_data(8));
    int16_t y = ((get_data(11) << 8) + get_data(10));
    int16_t z = ((get_data(13) << 8) + get_data(14));
    // NOTE: The axis are swaped on purpose so they match the global frame
    m->gyro.y = ((double)x / FS_SEL_0);
    m->gyro.x = ((double)y / FS_SEL_0);
    m->gyro.z = ((double)z / FS_SEL_0);

     // Filter values
    // Assume ~100Hz sample rate, 2nd order band pass butterworth filter
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

    filt[4].x = (c1 * m->gyro.x) + (c2 * filt[0].x) + (c3 * filt[1].x) + (c4 * filt[2].x)+ (c5 * filt[3].x);
    m->gyro.x = (filt[0].x + filt[4].x) - 2 * filt[2].x;
    filt[4].y = (c1 * m->gyro.y) + (c2 * filt[0].y) + (c3 * filt[1].y) + (c4 * filt[2].y)+ (c5 * filt[3].y);
    m->gyro.y = (filt[0].y + filt[4].y) - 2 * filt[2].y;
    filt[4].z = (c1 * m->gyro.z) + (c2 * filt[0].z) + (c3 * filt[1].z) + (c4 * filt[2].z)+ (c5 * filt[3].z);
    m->gyro.z = (filt[0].z + filt[4].z) - 2 * filt[2].z;

    x = ((get_data(1) << 8) + get_data(2));
    y = ((get_data(3) << 8) + get_data(4));
    z = ((get_data(5) << 8) + get_data(6));

    m->acc.y = ((double)x / AFS_SEL_0);
    m->acc.x = ((double)y / AFS_SEL_0);
    m->acc.z = ((double)z / AFS_SEL_0);
}

void mpu_init(void)
{
    DMPU("Starting mpu init...");
    write_register(USER_CTRL, (1 << 4)); // Disable i2c 
    transfer(MPU_ID);
    write_register(PWR_MGMT_1, 1 << 7); // Reset device
    transfer(MPU_ID);
    udelay(200000);

    write_register(USER_CTRL, (1 << 4)); // Disable i2c 
    transfer(MPU_ID);
    write_register(SAMPLE_RATE_DIV, 0x7); // 1kHz sample rate
    transfer(MPU_ID);
    write_register(PWR_MGMT_1, 0x1); // Set clock source to gyro x
    //write_register(INT_PIN_CFG, 1 << 5);// Latch interrrupts & clear interrupt on read
    //write_register(INT_ENABLE, 1); // Enable data ready interrupts
	transfer(MPU_ID);
    udelay(400000);

    int ret;
    int ret_i = read_register(WHO_AM_I_REG); 
    transfer(MPU_ID);
    if ((ret = get_data(ret_i)) != WHO_AM_I) {
        DMPU("Initilisation failed (who_am_i is %d)\n", ret);
    } else {
        DMPU("Init success!\n");
    }
    DMPU("Init finished!");

    mpu_read(&cal);
}
