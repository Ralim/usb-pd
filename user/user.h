
#ifndef USER_H_
#define USER_H_
#include <stdint.h>

/*
 * Functions that are left up to the end application to implement
 */

/*
 * Implement reading a memory register from the FUSB device over I2C
 * Ensure only the _size_ number of bytes are written to the provided buffer
 * to avoid memory corruption.
 *
 */
bool fusb_read_buf(uint8_t addr, uint8_t size, uint8_t *buf);

/*
 * Write multiple bytes to the FUSB302B
 *
 * addr: The memory address to which we will write
 * size: The number of bytes to write
 * buf: The buffer to write
 */
bool fusb_write_buf(uint8_t addr, uint8_t size, const uint8_t *buf);

#endif