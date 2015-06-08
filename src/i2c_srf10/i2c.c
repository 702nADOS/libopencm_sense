#include <libopencm3/stm32/i2c.h>
#include "i2c.h"

static void i2c_send_start_and_wait_for_rdy(uint32_t i2c) {
	/* start transaction */
	i2c_send_start(i2c);
	/* wait for START to finish and switch to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
}

static void i2c_send_address_rw(uint32_t i2c, uint8_t address, uint8_t rw) {
	uint32_t reg32 __attribute__((unused));
	/* say to what address we want to talk to. */
	/* yes, WRITE is correct - for selecting register */
	i2c_send_7bit_address(i2c, address, rw);
	/* enable ACK. */
	i2c_enable_ack(i2c);
	/* waiting for address is transferred. */
	while(!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	/* cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);
}

static void i2c_send_register(uint32_t i2c, uint8_t reg) {
	/* send desired register. */
	i2c_send_data(i2c, reg);
	while(!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
}

void i2c_read_buffer(uint32_t i2c, uint8_t address, uint8_t reg,
				uint8_t len, uint8_t *buffer) {
	i2c_send_start_and_wait_for_rdy(i2c);
	i2c_send_address_rw(i2c, address, I2C_WRITE);
	i2c_send_register(i2c, reg);

	/* send repeated start. */
	i2c_send_start_and_wait_for_rdy(i2c);
	/* send address again, but this time READ! */
	i2c_send_address_rw(i2c, address, I2C_READ);

	for(uint8_t i = 0; i < len; i++) {
		/* last byte transfer. */
		if(i == len - 1) {
			/* disbale ACK. */
			i2c_disable_ack(i2c);
			buffer[i] = i2c_get_data(i2c);
			/* NACK last byte. */
			i2c_nack_current(i2c);
		/* normal byte transfer. */
		} else {
			/* wait for byte transfer finish. */
			while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
			buffer[i] = i2c_get_data(i2c);
		}
	}

	/* send stop. */
	i2c_send_stop(i2c);
}

void i2c_write_buffer(uint32_t i2c, uint8_t address, uint8_t reg,
				uint8_t len, const uint8_t *buffer) {
	i2c_send_start_and_wait_for_rdy(i2c);
	i2c_send_address_rw(i2c, address, I2C_WRITE);
	i2c_send_register(i2c, reg);

	for(uint8_t i = 0; i < len; i++) {
		i2c_send_data(i2c, buffer[i]);
		if(i != len - 1)
			while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
		else
			while(!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
	}

	i2c_send_stop(i2c);
}
