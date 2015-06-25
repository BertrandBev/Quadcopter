/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef GYRO_H
#define GYRO_H

#include <spi_sensors_inf.h>

void gyro_init();
void gyro_read(gyro_val_t *g);

#endif
