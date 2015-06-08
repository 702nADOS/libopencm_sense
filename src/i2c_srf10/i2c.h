#ifndef _I2C_H_
#define _I2C_H_

#include <libopencm3/stm32/i2c.h>

void i2c_read_buffer(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t len, uint8_t *buffer);
void i2c_write_buffer(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t len, uint8_t *buffer);

#endif
