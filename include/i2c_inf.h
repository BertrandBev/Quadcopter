/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef I2C_INF_H
#define I2C_INF_H

#define I2C_MAX_TRANS_SIZE 255
#define PWM_ADDR 0x8E

typedef struct {
    uint8_t txbuf[I2C_MAX_TRANS_SIZE];
    uint8_t rxbuf[I2C_MAX_TRANS_SIZE];
} i2c_dev_port_t;

#endif
