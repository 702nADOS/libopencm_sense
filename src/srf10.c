#include <libopencm3/stm32/i2c.h>

void srf10_start_measurement(uint32_t i2c, uint8_t sensor, uint8_t mode) {
	uint32_t reg32 __attribute__((unused));

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
	i2c_send_7bit_address(i2c, sensor, I2C_WRITE);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);
	i2c_send_data(i2c, 0x00);
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	i2c_send_data(i2c, mode);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
	i2c_send_stop(i2c);
}

uint8_t srf10_measurement_successful(uint32_t i2c, uint8_t sensor) {
	uint32_t reg32 __attribute__((unused));
	uint8_t result;

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
	i2c_send_7bit_address(i2c, sensor, I2C_WRITE);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);
	i2c_send_data(i2c, 0x00);
	while (!(I2C_SR1(I2C1) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
	i2c_send_7bit_address(i2c, sensor, I2C_READ);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	result = i2c_get_data(i2c);
	while (!(I2C_SR1(I2C1) & (I2C_SR1_BTF | I2C_SR1_TxE)));
	i2c_send_stop(i2c);

	return result == 0xff;
}

uint16_t srf10_get_last_measurement(uint32_t i2c, uint8_t sensor) {
	uint32_t reg32 __attribute__((unused));
	uint16_t result;

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
	i2c_send_7bit_address(i2c, sensor, I2C_WRITE);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);
	i2c_send_data(i2c, 0x02);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
	i2c_send_7bit_address(i2c, sensor, I2C_READ);
	i2c_enable_ack(i2c);
	i2c_nack_next(i2c);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	result = (uint16_t)(i2c_get_data(i2c) << 8);
	i2c_disable_ack(i2c);
	result |= i2c_get_data(i2c);
	i2c_nack_current(i2c);
	i2c_send_stop(i2c);

	return result;
}

/*
 * range = (43mm * factor) + 43mm
 * factor = range(0x00, 0xff)
 */
void srf10_change_range(uint32_t i2c, uint8_t sensor, uint8_t factor) {
	uint32_t reg32 __attribute__((unused));

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
	i2c_send_7bit_address(i2c, sensor, I2C_WRITE);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);
	i2c_send_data(i2c, 0x02);
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	i2c_send_data(i2c, factor);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
	i2c_send_stop(i2c);
}

/*
 * factor = range(0, 16)
 */
void srf10_change_range(uint32_t i2c, uint8_t sensor, uint8_t factor) {
	uint32_t reg32 __attribute__((unused));

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
	i2c_send_7bit_address(i2c, sensor, I2C_WRITE);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);
	i2c_send_data(i2c, 0x01);
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	i2c_send_data(i2c, factor);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
	i2c_send_stop(i2c);
}
