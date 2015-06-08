#include <libopencm3/stm32/i2c.h>
#include "lsm9ds0.h"
#include "../i2c_srf10/i2c.h"

void lsm9ds0_setup_temp(uint32_t i2c, uint8_t sensor)
{
	uint8_t result;
	i2c_read_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG5_XM, 1, &result);
	/* i2c_write_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG5_XM, 1, 0b11110000); */
	i2c_write_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG5_XM, result | (1<<7));
	/* i2c_write8(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG5_XM, 0b11110000); */
	/* uint8_t tmpReg = i2c_read8(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG5_XM); */
	/* i2c_write8(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG5_XM, tmpReg | (1<<7)); */
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
