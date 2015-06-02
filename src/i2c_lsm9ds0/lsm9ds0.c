#include <libopencm3/stm32/i2c.h>
#include "lsm9ds0.h"

/* TODO: rename offset to register */

static void i2c_send_start_and_wait_for_rdy(uint32_t i2c)
{
     /* start transaction */
     i2c_send_start(i2c);
     /* Waiting for START is send and switched to master mode. */
     while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	      & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
}

static void i2c_send_address_rw(uint32_t i2c, uint8_t address, uint8_t rw)
{
     uint32_t reg32 __attribute__((unused));
     /* Say to what address we want to talk to. */
     /* Yes, WRITE is correct - for selecting register */
     i2c_send_7bit_address(i2c, address, rw);
     /* Waiting for address is transferred. */
     while(!(I2C_SR1(i2c) & I2C_SR1_ADDR));
     /* Cleaning ADDR condition sequence. */
     reg32 = I2C_SR2(i2c);
}

static void i2c_send_offset(uint32_t i2c, uint8_t offset)
{
     /* send offset to device where to read from */
     i2c_send_data(i2c, offset);
     while(!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
}

static void i2c_send_address_rw_ack(uint32_t i2c, uint8_t address, uint8_t rw)
{
     uint32_t reg32 __attribute__((unused));
     /* Say to what address we want to talk to. */
     /* Yes, WRITE is correct - for selecting register */
     i2c_send_7bit_address(i2c, address, rw);
     /* Acknowledgment */
     i2c_enable_ack(i2c);
     i2c_nack_next(i2c);
     while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
     reg32 = I2C_SR2(i2c);
}

void i2c_write8(uint32_t i2c, uint8_t address, uint8_t reg8, uint8_t value)
{
     i2c_send_start_and_wait_for_rdy(i2c);
     i2c_send_address_rw(i2c, address, I2C_WRITE);

     i2c_send_data(i2c, reg8);
     /* Wait until a data is received in DR register */
     while (!(I2C_SR1(i2c) & I2C_SR1_BTF));

     /* write value */
     i2c_send_data(i2c, value);
     while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

     /* stop transmission */
     i2c_send_stop(i2c);
}

void i2c_read_buffer(uint32_t i2c, uint8_t address, uint8_t reg8,
			uint8_t len, uint8_t *buffer)
{
     i2c_send_start_and_wait_for_rdy(i2c);
     i2c_send_address_rw(i2c, address, I2C_WRITE);
     i2c_send_offset(i2c, reg8);

     /* Now we send another START condition (repeated START) and then
      * transfer the destination but with flag READ.*/
     i2c_send_start_and_wait_for_rdy(i2c);
     /* Acknowledge */
     i2c_send_address_rw_ack(i2c, address, I2C_WRITE);

     for(uint8_t i = 0; i < len; i++) {
	  /* Wait until a data is received in DR register */
	  while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	  buffer[i] = i2c_get_data(i2c);
	  i2c_disable_ack(i2c);
     }

     /* end transmission */
     i2c_nack_current(i2c);
     i2c_send_stop(i2c);
}

uint8_t i2c_read8(uint32_t i2c, uint8_t address, uint8_t reg8)
{
     uint8_t value;

     i2c_read_buffer(i2c, address, reg8, 1, &value);

     return value;
}

void lsm9ds0_setup_temp(uint32_t i2c, uint8_t sensor)
{
     i2c_write8(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG5_XM, (1<<7));
}


uint16_t lsm9ds0_read_temp(uint32_t i2c, uint8_t sensor)
{
     uint8_t buffer[2];
     i2c_read_buffer(i2c, sensor, 0x80 | LSM9DS0_REGISTER_TEMP_OUT_L_XM,
		     2, buffer);
     uint16_t temperature = buffer[1];
     temperature <<= 8;
     temperature |= buffer[0];
     return temperature;
}
