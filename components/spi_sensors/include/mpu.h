/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef MPU_H
#define MPU_H

#include <spi_sensors_inf.h>

void mpu_init();
void mpu_read(mpu_val_t *m);

#endif
