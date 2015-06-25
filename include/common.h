/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef COMMON_H
#define COMMON_H

#define MOTOR_APP_ID    0
#define FLIGHT_APP_ID   1
#define GCC_APP_ID      2
#define PLANNER_APP_ID  3
#define NUM_APPS		4

/* Generic Constants */
#define TRUE 	1
#define FALSE	0

typedef enum{
	CLEAR 		= 2,
	ENABLE 		= 1,
	DISABLE 	= 0
}OPTION;

#define ROUND(a,b)		(((a) + (b) - 1) & ~((b) - 1))
#define DIV_ROUND(n,d)		(((n) + ((d)/2)) / (d))
#define DIV_ROUND_UP(n,d)	(((n) + (d) - 1) / (d))
#define roundup(x, y)		((((x) + ((y) - 1)) / (y)) * (y))

#endif
