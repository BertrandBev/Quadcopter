/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef CHANNEL_T_H
#define CHANNEL_T_H

#define MAX_CHANNELS 8

#define PULSE_WIDTH_US 300
#define MIN_DATA_WIDTH_US 900
#define MAX_DATA_WIDTH_US 2200
#define START_FRAME_MIN_US 3000

#define NO_SIGNAL -1
#define COUNT_INDEX 10

/**
 * Type for PPM channels
 */
typedef struct {
    double c[MAX_CHANNELS];
    int num_valid;
} ppm_channels_t;

#endif
