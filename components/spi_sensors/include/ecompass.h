/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef ECOMPASS_H
#define ECOMPASS_H

#include <spi_sensors_inf.h>

void ecompass_init();
void ecompass_read(ecompass_val_t *e);

#endif
