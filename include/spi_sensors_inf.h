/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef SPI_SENSORS_INF_H
#define SPI_SENSORS_INF_H

typedef struct {
    double temperture;
    double pressure;
} baro_val_t;

typedef struct {
    double x, y, z;
} vec3f_t;

typedef struct {
    vec3f_t mag, acc;
} ecompass_val_t;

typedef struct {
    vec3f_t acc, gyro;
} mpu_val_t;

typedef vec3f_t gyro_val_t;

typedef struct {
    baro_val_t baro;
    ecompass_val_t ecompass;
    mpu_val_t mpu;
    gyro_val_t gyro;
} sensors_t;

#endif

