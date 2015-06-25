/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <motors.h>
#include <stdio.h>
#include <utils.h>
#include <assert.h>
#include <common.h>
#include <i2c_inf.h>

// Uncomment if you want to see the leds instead of moving motors
//#define LED_DEBUG
//#define DEBUG_MOTORS
#ifdef DEBUG_MOTORS
#define DMOTORS(args...) \
    do { \
        printf("MOTORS %s(%d): ", __func__, __LINE__); \
        printf(args); \
        printf("\n"); \
    } while(0)
#else
#define DMOTORS(...) do{}while(0)
#endif

// Driver constants
#define MIN_PWM 225
#define MAX_PWM 410

#define MODE1 0x0
#define MODE2 0x1
#define ALL_LED_ON_L 0xFA
#define ALL_LED_ON_H 0xFB
#define ALL_LED_OFF_L 0xFC
#define ALL_LED_OFF_H 0xFD
#define PRESCALE 0xFE

#define FL 1
#define FR 0
#define BL 2
#define BR 3

#define LED0 0x06
#define LED1 0x0A
#define LED2 0x0E
#define LED3 0x12   

#define OUTDRV_BIT (1 << 2)
#define OUTNE_L_BIT (1 << 0)
#define AI_BIT (1 << 5)
#define INVRT_BIT (1 << 4) 

int wcount, rcount;
i2c_dev_port_t **i2c_port = (i2c_dev_port_t**)&i2c_buf;
static int safety_kill_on = FALSE;

void motors_quick_kill()
{
    // Choice here of turning off the enable pin or writing 0x0001XXXX to
    // register AL_LED_OFF_H
    // For now just set the motors to zero and flag that they can't be set again
    DMOTORS("Got quick kill signal");
    motors_set(0, 0, 0, 0);
    safety_kill_on = TRUE;
}

// Puts the data you want to send in the buffer and updates the
// amount to write
void write_byte(uint8_t value)
{
    (*i2c_port)->txbuf[wcount] = value;
    ++wcount;
}

void write_register(uint8_t address, uint8_t value)
{
    //wcount = 0;
    write_byte(address);
    write_byte(value);
}

// Assumes auto increment is on and the register point is at the start of
// the pwm channel you wish to set.
void set(double speed)
{
    int out = MIN_PWM + ((double)(MAX_PWM - MIN_PWM)) * speed;
    write_byte(0); // on_l
    write_byte(0); // on_h
    write_byte(out); // off_l
    write_byte(out >> 8); // off_h
}

double cap(double a) {
    double r = a;
    if (a < 0) r = 0;
    else if (a > 1) r = 1;
    return r;
}

void motors_set(double fl, double fr, double bl, double br)
{
    DMOTORS("Got values %lf %lf %lf %lf", fl, fr, bl, br);
   
    // Reset the position in the i2c buffer
    wcount = 0;
    rcount = 0;

    // Write the register to start the auto increment from
    write_byte(LED0);

    // Write the motor registers out
    set(cap(fr));
    set(cap(bl));
    set(cap(fl));
    set(cap(br));

    // Send the i2c transaction
    i2c_transfer(MOTOR_APP_ID, PWM_ADDR, wcount, rcount);
    
    DMOTORS("Finished setting motors");
    // Update the watchdog time
}

void motors__init(void)
{
    DMOTORS("Start init motor driver\n");
    
    // Reset the position in the i2c buffer
    wcount = 0;
    rcount = 0;
#ifdef LED_DEBUG
    write_register(MODE2, INVRT_BIT | OUTDRV_BIT | OUTNE_L_BIT); 
#else
    write_register(MODE2,  OUTDRV_BIT | OUTNE_L_BIT); // For driving motors don't invert signal
#endif
    i2c_transfer(MOTOR_APP_ID, PWM_ADDR, wcount, rcount);

    wcount = 0;
    write_register(PRESCALE, 135);
    i2c_transfer(MOTOR_APP_ID, PWM_ADDR, wcount, rcount);

    wcount = 0;
    write_register(MODE1, AI_BIT); // Clear sleep bit to start output
    i2c_transfer(MOTOR_APP_ID, PWM_ADDR, wcount, rcount);

    udelay(2000); // wait for internal clock to stablise

    // Need to set the motors to zero to disenage the safety
    // mode on the esc
    motors_set(0, 0, 0, 0);
    udelay(20000); // Wait a full servo period.
    DMOTORS("Finished init");
}
