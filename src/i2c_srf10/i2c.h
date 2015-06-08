#ifndef _I2C_H_
#define _I2C_H_

#include <libopencm3/stm32/i2c.h>

void i2c_read_buffer(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t len, uint8_t *buffer);
<<<<<<< HEAD
void i2c_write_buffer(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t len, const uint8_t *buffer);
=======
void i2c_write_buffer(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t len, uint8_t *buffer);

#endif
>>>>>>> f8996a1d3ad4a71a281c7700d1da652a48836787
