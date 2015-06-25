/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef BARO_H
#define BARO_H

#include <spi_sensors_inf.h>

void baro_init();
void baro_read(baro_val_t *b);

#endif
