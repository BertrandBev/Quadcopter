/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef FLIGHT_INF_H
#define FLIGHT_INF_H

/*
 * Type for passing quadcopter state
 */
typedef struct {
    double x, y, z;
    double roll, pitch, yaw;
} vector_6f_t;

#endif
