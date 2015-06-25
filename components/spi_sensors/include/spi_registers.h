/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */
#ifndef SPI_SENSORS_H
#define SPI_SENSORS_H

// Puts a single byte into the spi buffer
void write_byte(int value);
// Puts the commands to write to a register into the spi buffer
void write_register(int address, unsigned char value);
// Puts read command into the spi buffer and returns the index to read
// the data back from
int read_byte();
// Puts the command to read a register into the spi buffer and returns
// the index to read the data back from
int read_register();
// Gets the data out of the return spi buffer at the index specified
uint8_t get_data(int index);
// Transfer the spi buffer to the specified device
void transfer(int dev_id);
// Reset the buffer to the begining
void clear();
// Wrapper for logging - not sure how to do this nicely and be able to
// call the orginal log_it function. TODO: get rid of this hack
void spi_log(char *msg);
#endif
